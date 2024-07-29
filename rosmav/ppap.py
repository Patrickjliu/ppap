#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn
from time import sleep


class DanceNode(Node):
    forward = False
    back = False
    left = False
    right = False
    up = False
    down = False
    turn_left = False
    turn_right = False
    lightson = False
    lightsoff = True
    flash_light = False
    flip = False
    cork_left = False
    cork_weird_8 = False
    turn_left_up = False
    spin_up = False
    spiral_up = False
    small_figure_eight = False
    bop_up_n_down = False
    step_counter = 0


    def __init__(self):
        super().__init__("dancing_node")

        self.command_pub = self.create_publisher(
            OverrideRCIn,
            "bluerov2/override_rc",
            10
        )

        self.loop = self.create_timer(1, self._loop)

    def _set_neutral_all_channels(self):
        neutral = OverrideRCIn()
        neutral.channels = [1500] * 18
        self.command_pub.publish(neutral)

    def _loop(self):
        if self.step_counter > 50:
            self.destroy_node()
            return
        self._dance_moves()

        # See https://www.ardusub.com/developers/rc-input-and-output.html#rc-input

        commands = OverrideRCIn()
        commands.channels = [OverrideRCIn.CHAN_NOCHANGE] * 18

        if self.forward:    # forward
            commands.channels[4] = 1700
        elif self.back:     # back
            commands.channels[4] = 1300


        if self.left:       #left
            commands.channels[5] = 1300
        elif self.right:    #right
            commands.channels[5] = 1700


        if self.up:         # up
            commands.channels[2] = 1700
        elif self.down:     # down
            commands.channels[2] = 1300



        if self.turn_left:       # spin left
            commands.channels[3] = 1100
        elif self.turn_right:    # spin_right
            commands.channels[3] = 1900



        if self.lightson:        #lights on
            commands.channels[8] = 1500
            commands.channels[9] = 1500
        elif self.lightsoff:     #lights off
            commands.channels[8] = 1000
            commands.channels[9] = 1000


        if self.flash_light:
            #flashlight will keep light on after flashing,
            #use lightsoff to turn off lights
            for _ in range (6):
                commands.channels[8] = 1500
                commands.channels[9] = 1500
                sleep(1)
                commands.channels[8] = 1000
                commands.channels[9] = 1000
                sleep(1)
                commands.channels[8] = 1500
                commands.channels[9] = 1500



        if self.flip:
            commands.channels[3] = 1100
            commands.channels[5] = 1900

        if self.cork_left:
            commands.channels[3] = 1100
            commands.channels[5] = 1900
            commands.channels[3] = 1100
        if self.cork_weird_8:
            commands.channels[3] = 1100
            commands.channels[5] = 1900
            commands.channels[5] = 1100
            commands.channels[8] = 1500
            commands.channels[9] = 1500

        if self.turn_left_up:
            commands.channels[2] = 1900
            commands.channels[3] = 1200
            commands.channels[4] = 1900

        if self.spiral_up:
            commands.channels[3] = 1200
            commands.channels[2] = 1900
            commands.channels[4] = 1900

        if self.small_figure_eight:
            commands.channels[4] = 1600  # Forward
            commands.channels[3] = 1600  # Spin right
            commands.channels[5] = 1400  # Spin left
            sleep(.5)
            commands.channels[4] = 1600
            commands.channels[3] = 1400
            commands.channels[5] = 1200

        if self.bop_up_n_down:
            for _ in range (6):
                commands.channels[2] = 1000
                sleep(0.5)
                commands.channels[2] = 1500
                sleep(0.5)
                commands.channels[2] = 1000
                sleep(0.5)
                commands.channels[2] = 1500
                sleep(0.5)
                commands.channels[2] = 1000
                sleep(0.5)
                commands.channels[2] = 1500

        self.command_pub.publish(commands)


    def _dance_moves(self):
        self.step_counter += 1

        # Delay before starting the dance routine
        if self.step_counter < 6:
            sleep(0.1)

        # [0:06:25] - [0:08:25] - (down)
        elif self.step_counter < 8.3:
            self.down = True  # Set the down flag to true for 2 seconds


        # [0:08:25] - [0:09] - Turn left 360 degrees
        elif self.step_counter < 9:
            self.down = True
            self.turn_left = True
            sleep()

        # P-P-A-P [0:09] - [0:11]
        elif self.step_counter < 11:
            self.flash_light = True  # Turn lights on
            self.lightsoff = False
            self.turn_left = False
            self.down = False

        elif self.step_counter < 17:
            self.small_figure_eight = True  # Perform small figure-eight maneuver for 6 seconds
            self.flash_light = False
            self.lightson = True
            self.lightsoff = False
        #Assume during this time, robot floated up


        # I have a pen, I have an apple [0:17]
        elif self.step_counter < 19:
            self.lightson = False
            self.lightsoff = True
            self.small_figure_eight = False
            self.left = True

        elif self.step_counter < 21:
            self.left = False
            self.right = True


        # Uh! Apple-pen! [0:21]
        elif self.step_counter < 22:
            self.right = False
            self.lightson = True
            self.lightsoff = False

        elif self.step_counter <24:
            self.turn_left = True
            self.down = True

        # I have a pen, I have pineapple, [0:25]
        elif self.step_counter <26:
            self.lightoff = True
            self.lightson = False
            self.turn_left = False
            self.down = False
            self.forward = True

        elif self.step_counter <28:
            self.forward = False
            self.back = True

        #Uh! pineapple pen [0.28]
        elif self.step_counter <29:
            self.back = False
            self.lightson = True
            self.lightsoff = False

        elif self.step_counter <31:
            self.turn_left = True
            self.down = True

        # Apple-pen, pineapple-pen [0:31]
        elif self.step_counter< 33:
            self.flip = True
            self.down = False
            self.turn_left = False

        elif self.step_counter < 35:
            self.cork_weird_8  = True
            self.flip = False

        # Uh! pen-pinapple-apple-pen [0:35 - 0:39]
        elif self.step_counter <36:
            self.flash_light = True  # Activate flashlights

        elif self.step_counter <39:
            self.zigzag = True  # Perform zigzag maneuver
            self.down = True

        elif self.step_counter <43:
            self.spiral_up = True  # Spiral up
            self.lightsoff = True
            self.flash_light = False

        # Pen-pineapple-apple-pen [0:43]
        elif self.step_counter < 45:
            self.bob_up_n_down = True  # Flip 360 degrees
            self.down = True
            self.spiral_up = False

        elif self.step_counter <49:
            self.turn_left = True

        elif self.step_counter >50:
            self._set_neutral_all_channels()








          #DONE!








    def destroy_node(self):
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    danceNode = DanceNode()

    try:
        rclpy.spin(danceNode)
    except KeyboardInterrupt:
        pass
    finally:
        danceNode.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
