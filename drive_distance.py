git add .from robot import Robot, EncoderCounter
from pid_controller import PIController
import time
import logging
logger = logging.getLogger("drive_distance")

def drive_distance(bot, distance, speed=80):

    set_primary = bot.set_left
    primary_encoder = bot.left_encoder
    set_secondary = bot.set_right
    secondary_encoder = bot.right_encoder
    controller = PIController(proportional_constant=5, integral_constant=0.3)

    set_primary(speed)
    set_secondary(speed)
    while primary_encoder.pulse_count < distance or secondary_encoder.pulse_count < distance:
        time.sleep(0.01)

        error = primary_encoder.pulse_count - secondary_encoder.pulse_count
        adjustment = controller.get_value(error)

        set_primary(int(speed - adjustment))
        set_secondary(int(speed + adjustment))

        logger.debug(f"Enkodery: główny: {primary_encoder.pulse_count}, drugorzędny: {secondary_encoder.pulse_count}," 
                    f"uchyb:{error} korekta: {adjustment:.2f}")
        logger.info(f"Odległości: główny: {primary_encoder.distance_in_mm()} mm, drugorzędny: {secondary_encoder.distance_in_mm()} mm")

logging.basicConfig(level=logging.DEBUG)
bot = Robot()
distance_to_drive = 1000
distance_in_ticks = EncoderCounter.mm_to_ticks(distance_to_drive)
drive_distance(bot, distance_in_ticks)
