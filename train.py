import agent
import time
import serial
from pyvesc import SetDutyCycle, encode

# Servo control using direction.py approach
def crc16(data):
    crc = 0
    for b in data:
        crc ^= (b << 8) & 0xFFFF
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc & 0xFFFF

COMM_SET_SERVO_POS = 12

class Servo:
    """Servo control via VESC serial (direction.py approach)"""
    def __init__(self, port='/dev/ttyACM0', baudrate=115200, ser=None):
        self.ser = ser if ser is not None else serial.Serial(port, baudrate=baudrate, timeout=0.1)
        self._owns_serial = ser is None
        if self._owns_serial:
            time.sleep(0.2)

    def write(self, pos):
        """Write servo position [0.0, 1.0]"""
        pos = max(0.0, min(1.0, pos))
        val = int(pos * 1000)
        payload = bytes([COMM_SET_SERVO_POS, (val >> 8) & 0xFF, val & 0xFF])
        crc = crc16(payload)
        packet = bytes([0x02, len(payload)]) + payload + bytes([(crc >> 8) & 0xFF, crc & 0xFF, 0x03])
        self.ser.write(packet)

    def cleanup(self):
        self.write(0.5)  # Center
        if self._owns_serial:
            self.ser.close()


class VESCThrottle:
    """Motor throttle via VESC SetDutyCycle, matching ../car-code/LibGamepad/robocar.py."""
    def __init__(
        self,
        port='/dev/ttyACM0',
        baudrate=115200,
        max_duty_percent=10.0,
        min_forward_duty_percent=8.0,
        ser=None,
    ):
        self.ser = ser if ser is not None else serial.Serial(port, baudrate=baudrate, timeout=0.1)
        self._owns_serial = ser is None
        self.max_duty_percent = max_duty_percent
        self.min_forward_duty_percent = min_forward_duty_percent
        self.last_command_percent = 0.0
        if self._owns_serial:
            time.sleep(0.2)

    def write(self, accel, brake):
        accel = min(max(accel, 0.0), 1.0)
        brake = min(max(brake, 0.0), 1.0)
        command_percent = (accel - brake) * self.max_duty_percent
        if accel > brake and 0.0 < command_percent < self.min_forward_duty_percent:
            command_percent = self.min_forward_duty_percent
        self.write_percent(command_percent)

    def write_percent(self, command_percent):
        command_percent = max(-100.0, min(100.0, command_percent))
        self.last_command_percent = command_percent
        duty = int(command_percent * 1000)
        self.ser.write(encode(SetDutyCycle(duty)))

    def cleanup(self):
        self.write_percent(0.0)
        if self._owns_serial:
            self.ser.close()


class VESCRobocar:
    """Shared VESC serial connection for steering servo and motor throttle."""
    def __init__(
        self,
        port='/dev/ttyACM0',
        baudrate=115200,
        max_duty_percent=10.0,
        min_forward_duty_percent=8.0,
    ):
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=0.1)
        time.sleep(0.2)
        self.servo = Servo(ser=self.ser)
        self.throttle = VESCThrottle(
            ser=self.ser,
            max_duty_percent=max_duty_percent,
            min_forward_duty_percent=min_forward_duty_percent,
        )

    def cleanup(self):
        self.throttle.cleanup()
        self.servo.cleanup()
        self.ser.close()

# ESC control using Jetson GPIO. Loaded lazily so VESC-only runs do not emit GPIO warnings.
GPIO = None


def load_gpio():
    global GPIO
    if GPIO is not None:
        return GPIO
    try:
        import Jetson.GPIO as gpio
    except (ImportError, RuntimeError) as exc:
        raise RuntimeError(f"Jetson.GPIO unavailable ({exc}). ESC control will not work.") from exc
    GPIO = gpio
    return GPIO

class ESC:
    """ESC control via Jetson GPIO PWM"""
    def __init__(self, pin=33, frequency=50, forward_direction='high'):
        gpio = load_gpio()
        if forward_direction not in ('high', 'low'):
            raise ValueError("forward_direction must be 'high' or 'low'")
        self.pin = pin
        self.forward_direction = forward_direction
        self.neutral_duty_cycle = 7.5
        self.duty_range = 2.5
        self.last_duty_cycle = self.neutral_duty_cycle
        gpio.setmode(gpio.BOARD)
        gpio.setup(self.pin, gpio.OUT)
        self.pwm = gpio.PWM(self.pin, frequency)
        self.pwm.start(self.neutral_duty_cycle)
        time.sleep(2)

    def write(self, accel, brake):
        """
        Write ESC control values.
        accel: [0.0, 1.0] - forward throttle
        brake: [0.0, 1.0] - reverse throttle
        """
        duty_cycle = self.neutral_duty_cycle

        if accel > 0.0:
            accel = min(max(accel, 0.0), 1.0)
            direction = 1 if self.forward_direction == 'high' else -1
            duty_cycle = self.neutral_duty_cycle + (direction * accel * self.duty_range)
        elif brake > 0.0:
            brake = min(max(brake, 0.0), 1.0)
            direction = -1 if self.forward_direction == 'high' else 1
            duty_cycle = self.neutral_duty_cycle + (direction * brake * self.duty_range)
        self.last_duty_cycle = duty_cycle
        self.pwm.ChangeDutyCycle(duty_cycle)

    def cleanup(self):
        self.last_duty_cycle = self.neutral_duty_cycle
        self.pwm.ChangeDutyCycle(self.neutral_duty_cycle)
        time.sleep(0.5)
        self.pwm.stop()


def main_loop(
    sensors,
    state,
    values,
    car_esc,
    car_servo,
    mlp_model=None,
    enable_stuck_recovery=True,
    steer_override=None,
):
    """
    Main control loop for RC car using agent.py algorithms.

    Args:
        sensors: agent.sensors object with distance readings and speed
        state: agent.state object for tracking state
        values: agent.heuristic_value object with configuration
        car_esc: ESC instance for motor control
        car_servo: Servo instance for steering control
        mlp_model: Optional MLP model for friction estimation (can be None)
        enable_stuck_recovery: If True, reverse/brake when the car appears stuck.
                               Disable this when speed comes only from camera
                               optical flow, which is unreliable at standstill.
        steer_override: Optional steering value in [-1, 1] from camera guidance.

    Returns:
        output: agent.output object with control values
    """
    if len(sensors.dist) < 3:
        raise ValueError(
            "main_loop requires at least three distance readings: "
            "[left, center, right]"
        )

    estimated_turn = agent.estimate_turn(sensors.dist, values)

    if mlp_model is not None:
        friction = agent.estimate_friction(sensors, mlp_model, state)
    else:
        friction = 1.0
    lambda2_adjusted = agent.adjust_for_width(values.tau / (friction ** 2), track_width=1.5)
    if len(sensors.dist) == 3:
        steer = agent.compute_steer_simple(sensors.dist)
    else:
        steer = agent.compute_steer(sensors.dist, values, sensors, estimated_turn)
    if steer_override is not None:
        steer = max(-1.0, min(1.0, steer_override))
    target_speed = agent.compute_target_speed(sensors.dist[1], estimated_turn, friction, values, lambda2_adjusted)
    target_speed = agent.apply_danger_zone_speed(target_speed, state, values)
    accel, brake = agent.speed_to_pedal(sensors.x_speed, target_speed)
    encoder_pulses_tick = state.encoder_pulse_count
    state.encoder_pulse_count = 0

    if encoder_pulses_tick > 0:
        brake = agent.apply_abs(brake, sensors, values, encoder_pulses_tick)
        accel = agent.apply_asr(accel, encoder_pulses_tick, sensors, values)
    output = agent.output(steer, accel, brake)
    output = agent.handle_jump(sensors, output, values, state)
    if enable_stuck_recovery:
        output = agent.handle_stuck(sensors, output, state, values)

    if sensors.damage > state.prev_damage + 50:
        agent.record_danger_zone(state)
    state.prev_damage = sensors.damage
    state.lap_position += sensors.x_speed * values.dt
    state.prev_steer = output.steer

    # Apply to RC car hardware
    # Servo: convert steer [-1, 1] to position [0, 1]
    car_servo.write(output.steer_servo())
    car_esc.write(output.accel, output.brake)
    return output


def create_simple_sensors(dist_readings, current_speed):
    """
    Helper function to create a sensors object for RC car.

    Args:
        dist_readings: list of distance sensor readings (e.g., [left, center, right])
        current_speed: current speed estimate in m/s or similar units

    Returns:
        agent.sensors object
    """
    return agent.sensors(
        dist=dist_readings,
        x_speed=current_speed,
        y_speed=0.0,
        z_speed=0.0,
        wheel_v=[current_speed] * 4,
        rpm=0,
        gear=1,
        track_pos=0.0,
        damage=0.0,
        opp=[]
    )


def create_simple_state():
    """
    Helper function to create a state object for RC car.

    Returns:
        agent.state object with default values
    """
    return agent.state(
        friction=1.0,
        danger_zones=[],
        lap_position=0.0,
        prev_opp_dist=0.0,
        is_stuck=False,
        stuck_timer=0.0
    )
