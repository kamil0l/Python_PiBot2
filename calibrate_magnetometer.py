import vpython as vp
from robot_imu import RobotImu
from imu_settings import mag_offsets

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


import vpython as vp
import logging
import time
from robot_imu import RobotImu

logging.basicConfig(level=logging.INFO)
imu = RobotImu()


pr = vp.graph(xmin=0, xmax=60, scroll=True)
graph_pitch = vp.gcurve(color=vp.color.red, graph=pr)
graph_roll = vp.gcurve(color=vp.color.green, graph=pr)

xyz = vp.graph(xmin=0, xmax=60, scroll=True)
graph_x = vp.gcurve(color=vp.color.orange, graph=xyz)
graph_y = vp.gcurve(color=vp.color.cyan, graph=xyz)
graph_z = vp.gcurve(color=vp.color.purple, graph=xyz)

start = time.time()
while True:
    vp.rate(100)
    elapsed = time.time() - start
    pitch, roll = imu.read_accelerometer_pitch_and_roll()
    raw_accel = imu.read_accelerometer()
    graph_pitch.plot(elapsed, pitch)
    graph_roll.plot(elapsed, roll)
    print(f"Pochylenie: {pitch:.2f}, Przechylenie: {roll:.2f}")
    graph_x.plot(elapsed, raw_accel.x)
    graph_y.plot(elapsed, raw_accel.y)
    graph_z.plot(elapsed, raw_accel.z)
