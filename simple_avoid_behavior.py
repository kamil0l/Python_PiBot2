from robot import Robot
from time import sleep

class ObstacleAvoidingBehavior:

    def __init__(self, the_robot):
        self.robot = the_robot
        self.speed = 60

    def get_motor_speed(self, distance):

        if distance < 0.2:
            return -self.speed
        else:
            return self.speed

    def run(self):
        while True:

            left_distance = self.robot.left_distance_sensor.distance
            right_distance = self.robot.right_distance_sensor.distance
            print("Lewy: {l:.2f}, Prawy: {r:.2f}".format(l=left_distance, r=right_distance))

            left_speed = self.get_motor_speed(left_distance)
            self.robot.set_right(left_speed)
            right_speed = self.get_motor_speed(right_distance)
            self.robot.set_left(right_speed)

            sleep(0.05)

bot = Robot()
behavior = ObstacleAvoidingBehavior(bot)
behavior.run()
