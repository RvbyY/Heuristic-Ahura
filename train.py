import agent
import time
import serial

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
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=0.1)
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
        self.ser.close()

# ESC control via VESC serial (same approach as robocar.py)
# La précédente implémentation GPIO PWM était incorrecte — le moteur est
# piloté par un VESC qui accepte uniquement des commandes série, pas du
# PWM sur un pin GPIO. Le GPIO 33 n'était pas connecté au VESC, donc
# les commandes moteur partaient dans le vide et la voiture ne bougeait pas.
try:
    from pyvesc import SetDutyCycle, encode
    PYVESC_AVAILABLE = True
except ImportError:
    PYVESC_AVAILABLE = False
    print("Warning: pyvesc not available. ESC control will not work.")

class ESC:
    """
    Contrôle moteur via VESC port série.

    Envoie des commandes SetDutyCycle sur le port série, exactement
    comme robocar.py. accel et brake viennent de agent.speed_to_pedal()
    en [0.0, 1.0] et sont combinés en duty cycle signé [-100, 100].
    """
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        if not PYVESC_AVAILABLE:
            raise RuntimeError(
                "Le module 'pyvesc' n'est pas installé. "
                "Installe-le avec : pip3 install pyvesc"
            )
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=0.1)
        time.sleep(0.2)

    def write(self, accel, brake):
        """
        Envoie le duty cycle au VESC via port série.

        accel : [0.0, 1.0] — accélération avant  → duty +0..+100
        brake : [0.0, 1.0] — freinage/arrière    → duty -0..-100
        """
        accel = min(max(float(accel), 0.0), 1.0)
        brake = min(max(float(brake), 0.0), 1.0)
        pourcent = (accel - brake) * 100.0
        duty = int(pourcent * 1000)
        self.ser.write(encode(SetDutyCycle(duty)))

    def stop(self):
        self.ser.write(encode(SetDutyCycle(0)))

    def cleanup(self):
        self.stop()
        self.ser.close()


def main_loop(sensors, state, values, car_esc, car_servo, mlp_model=None):
    """
    Main control loop for RC car using agent.py algorithms.

    Args:
        sensors: agent.sensors object with distance readings and speed
        state: agent.state object for tracking state
        values: agent.heuristic_value object with configuration
        car_esc: ESC instance for motor control
        car_servo: Servo instance for steering control
        mlp_model: Optional MLP model for friction estimation (can be None)

    Returns:
        output: agent.output object with control values
    """
    estimated_turn = agent.estimate_turn(sensors.dist, values)

    if mlp_model is not None:
        friction = agent.estimate_friction(sensors, mlp_model, state)
    else:
        friction = 1.0
    lambda2_adjusted = agent.adjust_for_width(values.tau / (friction ** 2), track_width=1.5)
    steer = agent.compute_steer_simple(sensors.dist)
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
