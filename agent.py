import math
import yaml
import statistics
import time

class sensors:
    """
    Simplified sensors class for RC car.
    For RC car usage:
    - dist: list of distance sensor readings (can be simplified, e.g., [left, center, right])
    - x_speed: current speed estimate (can be from encoder or IMU)
    - y_speed: lateral speed (optional, can be 0.0 if not available)
    - z_speed: vertical acceleration (optional, can be 0.0 if not available)
    - wheel_v: wheel velocities (optional, can be [x_speed]*4 if not available)
    - rpm: motor RPM (optional, can be estimated from speed or set to 0)
    - gear: gear number (not used in RC car, can be 1)
    - track_pos: position on track (optional, can be 0.0)
    - damage: damage counter (optional, can be 0.0)
    - opp: opponent distances (not used in RC car, can be empty list)
    """
    def __init__(self, dist, x_speed, y_speed=0.0, z_speed=0.0,
                 wheel_v=None, rpm=0, gear=1, track_pos=0.0, damage=0.0, opp=None):
        self.dist = dist
        self.x_speed = x_speed
        self.y_speed = y_speed if y_speed is not None else 0.0
        self.z_speed = z_speed if z_speed is not None else 0.0
        self.wheel_v = wheel_v if wheel_v is not None else [x_speed] * 4
        self.rpm = rpm
        self.gear = gear
        self.track_pos = track_pos
        self.damage = damage
        self.opp = opp if opp is not None else []

class state:
    def __init__(self, friction, danger_zones, lap_position,
                 prev_opp_dist, is_stuck, stuck_timer):
        self.friction, self.danger_zones, self.lap_position  = friction, danger_zones, lap_position
        self.prev_opp_dist, self.is_stuck, self.stuck_timer = prev_opp_dist, is_stuck, stuck_timer
        self.encoder_pulse_count = 0
        self.prev_damage = 0.0
        self.is_airborne = False
        self.prev_steer = 0.0

class output:
    """
    Output class for RC car control.
    - steer: steering angle in range [-1.0, 1.0] where -1 is full left, +1 is full right
    - accel: acceleration in range [0.0, 1.0]
    - brake: braking in range [0.0, 1.0]

    For RC car conversion:
    - steer_servo(): converts steer to servo position [0.0, 1.0] for direction.py
    - speed_pwm(): converts accel/brake to PWM duty cycle [5.0, 10.0] for ESC control
    """
    def __init__(self, steer, accel, brake):
        self.steer = steer  # -1.0 to 1.0
        self.accel = accel  # 0.0 to 1.0
        self.brake = brake  # 0.0 to 1.0

    def steer_servo(self):
        """Convert steer [-1, 1] to servo position [0, 1] for direction.py make_servo_packet()"""
        return (self.steer + 1.0) / 2.0

    def speed_pwm(self):
        """
        Convert accel/brake to PWM duty cycle for ESC control.
        Returns duty cycle where:
        - 7.5 = neutral (stopped)
        - 7.5 to 10.0 = forward (accel)
        - 7.5 to 5.0 = reverse (brake)
        Matches the ESC.write() method in train.py
        """
        duty_cycle = 7.5
        if self.accel > 0.0:
            duty_cycle = 7.5 + (self.accel * 2.5)  # 7.5 to 10.0
        elif self.brake > 0.0:
            duty_cycle = 7.5 - (self.brake * 2.5)  # 7.5 to 5.0
        return duty_cycle



class heuristic_value:
    def __init__(self, cfg):
        self.max_dist, self.max_speed, self.min_speed = cfg['max_dist'], cfg['max_speed'], cfg['min_speed']
        self.theta, self.x1, self.x2, self.y1, self.y2 = cfg['theta'], cfg['x1'], cfg['x2'], cfg['y1'], cfg['y2']
        self.a_trap, self.b_trap, self.c_trap, self.d_trap = cfg['a_trap'], cfg['b_trap'], cfg['c_trap'], cfg['d_trap']
        self.e1, self.e2 = cfg['e1'], cfg['e2']
        self.lambda1, self.lambda2 = cfg['lambda1'], cfg['lambda2']
        self.tau, self.abs_slip, self.abs_range, self.abs_min_speed = cfg['tau'], cfg['abs_slip'], cfg['abs_range'], cfg['abs_min_speed']
        self.asr_slip, self.asr_range, self.asr_max_speed = cfg['asr_slip'], cfg['asr_range'], cfg['asr_max_speed']
        self.omega1, self.omega2, self.omega3, self.omega4, self.omega5 = cfg['omega1'], cfg['omega2'], cfg['omega3'], cfg['omega4'], cfg['omega5']
        self.danger_speed_factor, self.min_speed_factor, self.danger_look_ahead = cfg['danger_speed_factor'], cfg['min_speed_factor'], cfg['danger_look_ahead']
        self.z_air, self.z_ground = cfg['z_air'], cfg['z_ground']
        self.wheel_radius, self.encoder_ppr = cfg['wheel_radius'], cfg['encoder_ppr']
        self.dt = cfg['dt']

def estimate_turn(dist, values):
    """Estimate turn based on distance sensors"""
    d_neg1 = dist[-1]
    d_0 = dist[1]
    d_pos1 = dist[0]
    theta = math.radians(values.theta)

    if d_pos1 > d_0:
        k = math.sin(theta) * d_pos1 / (d_0 - math.cos(theta) * d_pos1)
    else:
        k = math.sin(theta) * d_0 / (d_neg1 - math.cos(theta) * d_0)
    return math.atan(k)

def trapezoid(a, b, c, d, x):
    if x <= a:
        return 0.0
    elif x <= b:
        return (x - a) / (b - a)
    elif x <= c:
        return 1.0
    elif x <= d:
        return (c - x) / (d - c) + 1.0
    else:
        return 0.0

def clamp(n, min, max):
    if n < min:
        return min
    elif n > max:
        return max
    else:
        return n

def log_sigmoid(x1, x2, y1, y2, x):
    a = y2 - y1
    d = y1
    b = (math.log(a / (1.01 * y1 - d) - 1) - math.log(a / (0.99 * y2 - d) - 1)) / (x1 - x2)
    c = math.log(a / (1.01 * y1 - d) - 1) / b - x1
    return a / (1 + math.exp(b * (x + c))) + d

def compute_steer(dist, values, sensors, estimated_turn):
    """Compute steering using distance sensors"""
    base_sensor = dist.index(max(dist))
    y = log_sigmoid(values.x1, values.x2, values.y1, values.y2, sensors.dist[1])
    y = round(y)
    sensor_list = range(base_sensor - y, base_sensor + y + 1)
    sensor_list = [clamp(s, -9, 9) for s in sensor_list]

    if abs(estimated_turn) > 0.1:
        beta = trapezoid(values.a_trap, values.b_trap, values.c_trap, values.d_trap, sensors.dist[1])
        s = -1 if estimated_turn < 0 else 1
        alpha = 4 ** (s * beta)
    else:
        alpha = 1.0
    h = 0.0
    g = 0.0
    angle_base = math.radians((base_sensor - 1) * -45)
    for i in sensor_list:
        angle_i = math.radians((i - 1) * -45)
        dist_i = sensors.dist[i]
        if i == base_sensor:
            weight = 2 * dist_i
        elif angle_i > angle_base:
            weight = dist_i / alpha
        else:
            weight = alpha * dist_i
        h += weight * math.cos(angle_i)
        g += weight * math.sin(angle_i)
    raw_steer = math.atan(g / h)
    return clamp(raw_steer / (math.pi / 2), -1.0, 1.0)

def compute_steer_simple(dist):
    left, center, right = dist
    diff = right - left
    total = left + right
    if total <= 0.0:
        return 0.0
    steer = diff / total
    return clamp(steer, -1.0, 1.0)

def compute_target_speed(dist0, estimated_turn, friction, values, lambda2_adjusted):
    lambda_base = log_sigmoid(values.e1, values.e2, values.lambda1, lambda2_adjusted, estimated_turn)
    friction_factor = values.tau / (friction ** 2)
    lambda_val = lambda_base * friction_factor / values.tau
    ratio = clamp(dist0 / values.max_dist, 0.0, 1.0)
    target_speed = (ratio ** lambda_val) * (values.max_speed - values.min_speed) + values.min_speed
    return target_speed

def speed_to_pedal(x_speed, target_speed):
    """Convert target speed to accel/brake pedals"""
    b = 1.0
    p = 2.0 / (1.0 + math.exp(b * (x_speed - target_speed))) - 1.0

    if p >= 0:
        accel = p
        brake = 0.0
    else:
        accel = 0.0
        brake = abs(p)
    return accel, brake

def apply_abs(brake, sensors, values, encoder_pulses_this_tick):
    pulse_rate = encoder_pulses_this_tick / values.dt
    v_i = (pulse_rate / values.encoder_ppr * (2 * math.pi)) * values.wheel_radius

    if v_i < values.abs_min_speed:
        return brake
    slip = abs(sensors.x_speed - v_i)
    if slip > values.abs_slip:
        correction = (slip - values.abs_slip) / values.abs_range
        brake = brake - correction
        brake = max(brake, 0.0)
    return brake

def apply_asr(accel, encoder_pulses_this_tick, sensors, values):
    pulse_rate = encoder_pulses_this_tick / values.dt
    v_i = (pulse_rate / values.encoder_ppr * (2 * math.pi)) * values.wheel_radius

    if v_i > values.asr_max_speed:
        return accel
    slip = abs(sensors.x_speed - v_i)
    if slip > values.asr_slip:
        correction = (slip - values.asr_slip) / values.asr_range
        accel -= correction
        accel = max(accel, 0.0)
    return accel

def estimate_friction(sensors, mlp_model, state):
    """Estimate friction (simplified if no MLP model)"""
    slip_d = abs(sensors.x_speed - statistics.mean(sensors.wheel_v))

    if abs(state.prev_steer) > 0.05:
        return state.friction
    if sensors.rpm < 7000:
        return state.friction
    features = [slip_d, sensors.rpm, sensors.z_speed]
    new_friction = mlp_model.predict(features)
    state.friction = 0.9 * state.friction + 0.1 * new_friction
    return state.friction

def adjust_for_width(lambda2, track_width):
    width_factor = 10.0 / max(track_width, 1.0)
    return lambda2 * width_factor

def handle_jump(sensors, output, values, state):
    z_accel = sensors.z_speed

    if z_accel > values.z_air:
        state.is_airborne = True
    if state.is_airborne and z_accel < values.z_ground:
        state.is_airborne = False
    if state.is_airborne:
        output.steer = 0.0
        output.accel = min(output.accel, 0.3)
    return output

def find_zone_near(danger_zones, pos, radius):
    for zone in danger_zones:
        if abs(zone['position'] - pos) <= radius:
            return zone
    return None

def record_danger_zone(state):
        pos = state.lap_position
        existing = find_zone_near(state.danger_zones, pos, radius=0.3)

        if existing:
            existing['severity'] += 1
        else:
            state.danger_zones.append({'position': pos, 'severity': 1})

def apply_danger_zone_speed(target_speed, state, values):
    upcoming = [z for z in state.danger_zones if 0 < (z['position'] - state.lap_position) < values.danger_look_ahead]

    if len(upcoming) == 0:
        return target_speed
    worst = max(upcoming, key=lambda z: z['severity'])
    factor = max(values.danger_speed_factor ** worst['severity'], values.min_speed_factor)
    return target_speed * factor

def handle_stuck(sensors, output, state, values):
    if abs(sensors.x_speed) < 0.05 and output.accel > 0.3:
        state.stuck_timer += values.dt
    else:
        state.stuck_timer = 0
    if state.stuck_timer > 1.0:
        state.is_stuck = True
    if state.is_stuck:
        output.accel = 0.0
        output.brake = 0.5
        output.steer = -output.steer
    if abs(sensors.x_speed) > 0.2:
        state.is_stuck = False
        state.stuck_timer = 0
    return output
