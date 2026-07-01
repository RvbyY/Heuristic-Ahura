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
try:
    from pyvesc import SetDutyCycle, encode
    PYVESC_AVAILABLE = True
except ImportError:
    PYVESC_AVAILABLE = False
    print("Warning: pyvesc not available. ESC control will not work.")

class ESC:
    """
    Contrôle moteur via VESC port série.
    accel et brake en [0.0, 1.0], combinés en duty cycle signé [-100, 100].
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


def main_loop(sensors, state, values, car_esc, car_servo, duty_cycle=0.15, mlp_model=None):
    """
    Boucle de contrôle principale.

    La vitesse est fixée directement par duty_cycle (0.0 à 1.0).
    L'algo de vitesse original (speed_to_pedal) est désactivé car il
    interprète les lignes blanches comme des obstacles et freine — ce qui
    est le comportement inverse de ce qu'on veut sur un circuit.

    Seule la direction est calculée par l'algo (compute_steer_simple +
    correction center_offset).

    Args:
        duty_cycle: puissance moteur fixe en [0.0, 1.0]. Commence à 0.15
                    et monte par paliers de 0.05 selon le comportement réel.
    """
    estimated_turn = agent.estimate_turn(sensors.dist, values)

    # Direction uniquement — vitesse gérée par duty_cycle fixe
    steer = agent.compute_steer_simple(sensors.dist)

    output = agent.output(steer, accel=duty_cycle, brake=0.0)
    output = agent.handle_stuck(sensors, output, state, values)

    state.lap_position += sensors.x_speed * values.dt
    state.prev_steer = output.steer

    # Envoi servo (direction)
    car_servo.write(output.steer_servo())
    # Envoi ESC (vitesse fixe — ne dépend pas des lignes détectées)
    car_esc.write(duty_cycle, 0.0)

    return output


def create_simple_sensors(dist_readings, current_speed):
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
    return agent.state(
        friction=1.0,
        danger_zones=[],
        lap_position=0.0,
        prev_opp_dist=0.0,
        is_stuck=False,
        stuck_timer=0.0
    )