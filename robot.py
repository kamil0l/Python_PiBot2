from Raspi_MotorHAT import Raspi_MotorHAT
from gpiozero import DistanceSensor

import atexit
import leds_led_shim
from servos import Servos
from encoder_counter import EncoderCounter


class Robot:
    wheel_diameter_mm = 70.0
    ticks_per_revolution = 40.0
    wheel_distance_mm = 132.0

    def __init__(self, motorhat_addr=0x6f):

        self._mh = Raspi_MotorHAT(addr=motorhat_addr)

        self.left_motor = self._mh.getMotor(1)
        self.right_motor = self._mh.getMotor(2)


        self.leds = leds_led_shim.Leds()

        self.servos = Servos(addr=motorhat_addr)


        self.left_distance_sensor = DistanceSensor(echo=17, trigger=27, queue_len=2)
        self.right_distance_sensor = DistanceSensor(echo=5, trigger=6, queue_len=2)


        EncoderCounter.set_constants(self.wheel_diameter_mm, self.ticks_per_revolution)
        self.left_encoder = EncoderCounter(4)
        self.right_encoder = EncoderCounter(26)


        atexit.register(self.stop_all)

    def convert_speed(self, speed):

        mode = Raspi_MotorHAT.RELEASE
        if speed > 0:
            mode = Raspi_MotorHAT.FORWARD
        elif speed < 0:
            mode = Raspi_MotorHAT.BACKWARD


        output_speed = (abs(speed) * 255) // 100
        return mode, int(output_speed)

    def set_left(self, speed):
        mode, output_speed = self.convert_speed(speed)
        self.left_motor.setSpeed(output_speed)
        self.left_motor.run(mode)

    def set_right(self, speed):
        mode, output_speed = self.convert_speed(speed)
        self.right_motor.setSpeed(output_speed)
        self.right_motor.run(mode)

    def stop_motors(self):
        self.left_motor.run(Raspi_MotorHAT.RELEASE)
        self.right_motor.run(Raspi_MotorHAT.RELEASE)

    def stop_all(self):
        self.stop_motors()


        self.leds.clear()
        self.leds.show()


        self.left_encoder.stop()
        self.right_encoder.stop()


        self.servos.stop_all()

    def set_pan(self, angle):
        self.servos.set_servo_angle(1, angle)

    def set_tilt(self, angle):
        self.servos.set_servo_angle(0, angle)

logging.basicConfig(level=logging.INFO)
imu = RobotImu()
robot_view()

accel_arrow = vp.arrow(axis=vp.vector(0, 1, 0))
x_arrow = vp.arrow(axis=vp.vector(1, 0, 0), color=vp.color.red)
y_arrow = vp.arrow(axis=vp.vector(0, 1, 0), color=vp.color.green)
z_arrow = vp.arrow(axis=vp.vector(0, 0, 1), color=vp.color.blue)

while True:
    vp.rate(100)
    accel = imu.read_accelerometer()
    print(f"Akcelerometr: {accel}")
    accel_arrow.axis = accel.norm()


r = robot.Robot()
r.set_left(100)
r.set_right(70)
sleep(1)


def straight(bot, seconds):
    bot.set_left(100)
    bot.set_right(100)
    sleep(seconds)

def turn_left(bot, seconds):
    bot.set_left(20)
    bot.set_right(80)
    sleep(seconds)

def turn_right(bot, seconds):
    bot.set_left(80)
    bot.set_right(20)
    sleep(seconds)

def spin_left(bot, seconds):
    bot.set_left(-100)
    bot.set_right(100)
    sleep(seconds)

bot = robot.Robot()
straight(bot, 1)
turn_right(bot, 0.6)
straight(bot, 0.6)
turn_left(bot, 0.6)
straight(bot, 0.6)
turn_left(bot, 0.6)
straight(bot, 0.3)
spin_left(bot, 1)


gyro_min = vp.vector(0, 0, 0)
gyro_max = vp.vector(0, 0, 0)

for n in range(500):
    gyro = imu.read_gyroscope()
    gyro_min.x = min(gyro_min.x, gyro.x)
    gyro_min.y = min(gyro_min.y, gyro.y)
    gyro_min.z = min(gyro_min.z, gyro.z)

    gyro_max.x = max(gyro_max.x, gyro.x)
    gyro_max.y = max(gyro_max.y, gyro.y)
    gyro_max.z = max(gyro_max.z, gyro.z)

    offset = (gyro_min + gyro_max) / 2

    time.sleep(.01)

print(f"Odchylenie od zera: {offset}.")

imu = RobotImu(mag_offsets=mag_offsets)

mag_min = vp.vector(0, 0, 0)
mag_max = vp.vector(0, 0, 0)


scatter_xy = vp.gdots(color=vp.color.red)
scatter_yz = vp.gdots(color=vp.color.green)
scatter_zx = vp.gdots(color=vp.color.blue)

while True:
    vp.rate(100)
    mag = imu.read_magnetometer()

    mag_min.x = min(mag_min.x, mag.x)
    mag_min.y = min(mag_min.y, mag.y)
    mag_min.z = min(mag_min.z, mag.z)

    mag_max.x = max(mag_max.x, mag.x)
    mag_max.y = max(mag_max.y, mag.y)
    mag_max.z = max(mag_max.z, mag.z)
    offset = (mag_max + mag_min) / 2

    print(f"Magnetometr: {mag}. Offsets: {offset}.")
    scatter_xy.plot(mag.x, mag.y)
    scatter_yz.plot(mag.y, mag.z)
    scatter_zx.plot(mag.z, mag.x)

imu = RobotImu(mag_offsets=imu_settings.mag_offsets,
               gyro_offsets=imu_settings.gyro_offsets)
fusion = ImuFusion(imu)
timer = DeltaTimer()
pid = PIController(0.7, 0.01)
robot = Robot()
base_speed = 70


heading_set_point = 0

while True:
    dt, elapsed = timer.update()
    fusion.update(dt)
    heading_error = fusion.yaw - heading_set_point
    steer_value = pid.get_value(heading_error, delta_time=dt)
    print(f"Błąd: {heading_error}, Wartość:{steer_value:2f}, czas: {elapsed}")
    robot.set_left(base_speed + steer_value)
    robot.set_right(base_speed - steer_value)


import time
from multiprocessing import Process, Queue

from flask import Flask, render_template, Response, request

app = Flask(__name__)
control_queue = Queue()
display_queue = Queue(maxsize=2)
display_template = 'image_server.html'


@app.after_request
def add_header(response):
    response.headers['Cache-Control'] = "no-cache, no-store, must-revalidate"
    return response


@app.route('/')
def index():
    return render_template(display_template)


def frame_generator():
    while True:
        time.sleep(0.05)
        encoded_bytes = display_queue.get()
        yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + encoded_bytes + b'\r\n')


@app.route('/display')
def display():
    return Response(frame_generator(),
        mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/control', methods=['POST'])
def control():
    control_queue.put(request.form)
    return Response('queued')


def start_server_process(template_name):
    """Uruchomienie procesu, wywołanie metody .terminate w celu zamknięcia"""
    global display_template
    display_template = template_name
    server = Process(target=app.run, kwargs={"host": "0.0.0.0", "port": 5001})
    server.start()
    return server


def put_output_image(encoded_bytes):
    """Kolejka obrazów wyjściowych"""
    if display_queue.empty():
        display_queue.put(encoded_bytes)


def get_control_instruction():
    if control_queue.empty():
        return None
    else:
        return control_queue.get()


from flask import Flask, render_template, Response
import camera_stream
import time

app = Flask(__name__)

@app.route('/')
def index():
    return render_template('image_server.html')

def frame_generator():
    """To jest nasze główne źródło obrazu z kamery"""
    camera = camera_stream.setup_camera()

    # allow the camera to warm up
    time.sleep(0.1)
    for frame in camera_stream.start_stream(camera):
        encoded_bytes = camera_stream.get_encoded_bytes_for_frame(frame)
        yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + encoded_bytes + b'\r\n')

@app.route('/display')
def display():
    return Response(frame_generator(),
        mimetype='multipart/x-mixed-replace; boundary=frame')

app.run(host="0.0.0.0", debug=True, port=5001)


from vpython import vector
gyro_offsets = vector(-0.583969, 0.675573, -0.530534)
mag_offsets = vector(7.725, 9.6, -25.275)



import colorsys


def show_rainbow(leds, led_range):
    led_range = list(led_range)
    hue_step = 1.0 / len(led_range)
    for index, led_address in enumerate(led_range):
        hue = hue_step * index
        rgb = colorsys.hsv_to_rgb(hue, 1.0, 0.6)
        rgb = [int(c*255) for c in rgb]
        leds.set_one(led_address, rgb)


from robot import Robot
from time import sleep

bot = Robot()
red = (255, 0, 0)
blue = (0, 0, 255)

while True:
    print("czerwony")
    bot.leds.set_all(red)
    bot.leds.show()
    sleep(0.5)
    print("niebieski")
    bot.leds.set_all(blue)
    bot.leds.show()
    sleep(0.5)


logging.basicConfig(level=logging.INFO)
imu = RobotImu()
robot_view()

mag_arrow = vp.arrow(pos=vp.vector(0, 0, 0))
x_arrow = vp.arrow(axis=vp.vector(1, 0, 0), color=vp.color.red)
y_arrow = vp.arrow(axis=vp.vector(0, 1, 0), color=vp.color.green)
z_arrow = vp.arrow(axis=vp.vector(0, 0, 1), color=vp.color.blue) #/// jednak musi być

while True:
    vp.rate(100)

    mag = imu.read_magnetometer()
    mag_arrow.axis = mag.norm()
    print(f"Magnetometr: {mag}")