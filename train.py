import agent
import time
from servo import Servo
import Jetson.GPIO as GPIO
import time

class ESC:
    def __init__(self, pin=33, frequency=50):
        self.pin = pin
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pin, frequency)
        self.pwm.start(7.5)
        time.sleep(2)

    def write(self, accel, brake):
        duty_cycle = 7.5

        if accel > 0.0:
            accel = min(max(accel, 0.0), 1.0)
            duty_cycle = 7.5 + (accel * 2.5)
        elif brake > 0.0:
            brake = min(max(brake, 0.0), 1.0)
            duty_cycle = 7.5 - (brake * 2.5)
        self.pwm.ChangeDutyCycle(duty_cycle)

    def cleanup(self):
        self.pwm.ChangeDutyCycle(7.5)
        time.sleep(0.5)
        self.pwm.stop()

def main_loop(sensors, state, mlp_model, values, car_esc, car_servo):
    estimated_turn = agent.estimate_turn(sensors.dist, values)
    friction = agent.estimate_friction(sensors, mlp_model, state)
    lambda2_adjusted = agent.adjust_for_width(values.tau / (friction ** 2), track_width=1.5)

    steer = agent.compute_steer(sensors.dist, values, sensors, estimated_turn, alpha=1.0)
    target_speed = agent.compute_target_speed(sensors.dist[1], estimated_turn, friction, values, lambda2_adjusted)
    target_speed = agent.apply_danger_zone_speed(target_speed, state, values)

    accel, brake = agent.speed_to_pedal(sensors.x_speed, target_speed)
    encoder_pulses_tick = state.encoder_pulse_count
    state.encoder_pulse_count = 0

    brake = agent.apply_abs(brake, sensors, values, encoder_pulses_tick)
    accel = agent.apply_asr(accel, encoder_pulses_tick, sensors, values)
    output = agent.output(steer, accel, brake)
    output = agent.handle_jump(sensors, output, values, state)
    output = agent.handle_stuck(sensors, output, state, values)

    if sensors.damage > state.prev_damage + 50:
        agent.record_danger_zone(state)
    state.prev_damage = sensors.damage
    state.lap_position += sensors.x_speed * values.dt

    car_servo.write(output.steer)
    car_esc.write(output.accel, output.brake)
    state.prev_steer = output.steer
    return output
