from Raspi_MotorHAT.Raspi_PWM_Servo_Driver import PWM
import atexit

pwm = PWM(0x6f)

pwm_frequency = 100
pwm.setPWMFreq(pwm_frequency)

servo_mid_point_ms = 1.5

deflect_90_in_ms = 0.5

period_in_ms = 1000 / pwm_frequency

pulse_steps = 4096

steps_per_ms = pulse_steps / period_in_ms

steps_per_degree = (deflect_90_in_ms * steps_per_ms) / 90

servo_mid_point_steps = servo_mid_point_ms * steps_per_ms

def convert_degrees_to_steps(position):
    return int(servo_mid_point_steps + (position * steps_per_degree))

atexit.register(pwm.setPWM, 0, 0, 4096)

while True:
    position = int(input("Wpisz pozycję w stopniach (90 do -90, 0 to środek): "))
    end_step = convert_degrees_to_steps(position)
    pwm.setPWM(0, 0, end_step)
