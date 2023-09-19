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


import ledshim


class Leds:
    @property
    def count(self):
        return ledshim.width

    def set_one(self, led_number, color):
        ledshim.set_pixel(led_number, *color)

    def set_range(self, led_range, color):
        for pixel in led_range:
            ledshim.set_pixel(pixel, *color)

    def set_all(self, color):
        ledshim.set_all(*color)

    def clear(self):
        ledshim.clear()

    def show(self):
        ledshim.show()
