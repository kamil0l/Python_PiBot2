from Raspi_MotorHAT.Raspi_PWM_Servo_Driver import PWM

class Servos:


    def __init__(self, addr=0x6f, deflect_90_in_ms = 0.5):

        self._pwm = PWM(addr)

        pwm_frequency = 100
        self._pwm.setPWMFreq(pwm_frequency)

        servo_mid_point_ms = 1.5

        period_in_ms = 1000 / pwm_frequency

        pulse_steps = 4096

        steps_per_ms = pulse_steps / period_in_ms

        self.steps_per_degree = (deflect_90_in_ms * steps_per_ms) / 90

        self.servo_mid_point_steps = servo_mid_point_ms * steps_per_ms

        self.channels = [0, 1, 14, 15]

    def stop_all(self):

        off_bit = 4096
        self._pwm.setPWM(self.channels[0], 0, off_bit)
        self._pwm.setPWM(self.channels[1], 0, off_bit)
        self._pwm.setPWM(self.channels[2], 0, off_bit)
        self._pwm.setPWM(self.channels[3], 0, off_bit)

    def _convert_degrees_to_steps(self, position):
        return int(self.servo_mid_point_steps + (position * self.steps_per_degree))

    def set_servo_angle(self, channel, angle):

        if angle > 90 or angle < -90:
            raise ValueError("Kąt spoza zakresu")

        off_step = self._convert_degrees_to_steps(angle)
        self._pwm.setPWM(self.channels[channel], 0, off_step)

