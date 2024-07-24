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
    lightsoff = False
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
            OverrideRCIn, "bluerov2/override_rc", 10
        )

        self.loop = self.create_timer(0.1, self._loop)

    def _set_neutral_all_channels(self):
        neutral = OverrideRCIn()
        neutral.channels = [1500] * 8
        self.command_pub.publish(neutral)

    def _loop(self):
        if self._step_counter > 60:
            self.destroy_node()
            return
        self._dance_moves()

        # See https://www.ardusub.com/developers/rc-input-and-output.html#rc-input

        commands = OverrideRCIn()
        commands.channels = [OverrideRCIn.CHAN_NOCHANGE] * 8

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
            commands.channels[3] = 1300
        elif self.turn_right:    # spin_right
            commands.channels[3] = 1700


            
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
            commands.channels[5] = 1100
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
            
        if self.bop_up_n_down
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
        self._step_counter += 0.1

        # Delay before starting the dance routine
        if self.step_counter < 6.2:
            sleep(0.1)

        # [0:06:25] - [0:08:25] - (down)
        elif self.step_counter < 8.2:
            self.down = True  # Set the down flag to true for 2 seconds


        # [0:08:25] - [0:09] - Turn left 360 degrees
        elif self.step_counter < 9.0:
            self.down = False
            self.turn_left = True

        # P-P-A-P [0:09] - [0:11]
        elif self.step_counter < 11:
            self.flash_light = True  # Turn lights on
            self.turn_left = False
            
        elif self.step_counter < 17:
            self.small_figure_eight = True  # Perform small figure-eight maneuver for 6 seconds
            self.flash_light = False
            self.lightson = True

        # I have a pen, I have an apple [0:17]
        elif self.step_counter <
        self.small_figure_eight = False
        self.up = True  # Move up
        sleep(1)
        self._set_neutral_all_channels()
        self.up = False
        self._set_neutral_all_channels()
        sleep(0.5)

        self.down = True  # Move down
        sleep(1)
        self._set_neutral_all_channels()
        self.down = False
        self._set_neutral_all_channels()
        sleep(1)

        self.back = True  # Move back
        sleep(0.2)
        self._set_neutral_all_channels()
        self._move_back = False
        self._set_neutral_all_channels()
        sleep(0.2)

        # Uh! Apple-pen! [0:21]
        self._turn_left = True  # Spin left 360 degrees
        sleep(4)
        self._set_neutral_all_channels()
        self._turn_left = False
        self._set_neutral_all_channels()
        sleep(1)

        self.spiral_up = True  # Spiral up
        sleep(3)  # Assuming spiral up for 3 seconds
        self._set_neutral_all_channels()
        self.spiral_up = False
        self._set_neutral_all_channels()
        sleep(0.6)

        # Flash lights and move forward [0:24]
        self.flash_light = True  # Activate flashlights
        sleep(2)  # Flashlights on for 2 seconds
        self._move_forward = True  # Move forward
        sleep(1)  # Move forward for 1 second
        self._set_neutral_all_channels()
        self.flash_light = False
        self._set_neutral_all_channels()
        self._move_forward = False
        self._set_neutral_all_channels()
        sleep(0.5)

        self._move_down = True  # Move down
        sleep(0.7)
        self._set_neutral_all_channels()
        self._move_down = False
        self._set_neutral_all_channels()
        sleep(0.7)

        # Uh! Pineapple-pen! [0:28]
        self._turn_left = True  # Spin left 360 degrees
        sleep(4)
        self._set_neutral_all_channels()
        self._turn_left = False
        self._set_neutral_all_channels()
        sleep(0.3)

        self._turn_left = True  # Spin left 360 degrees again
        sleep(4)
        self._set_neutral_all_channels()
        self._turn_left = False
        self._set_neutral_all_channels()
        sleep(0.3)

        # Apple-pen, pineapple-pen [0:31]
        self.cork_left = True  # Perform corkscrew maneuver to the left
        sleep(4)  # Assuming corkscrew maneuver takes 4 seconds
        self._set_neutral_all_channels()
        self.cork_left = False
        self._set_neutral_all_channels()
        sleep(1)

        self.cork_weird_8 = True  # Perform weird 8 maneuver
        sleep(4)  # Assuming weird 8 maneuver takes 4 seconds
        self._set_neutral_all_channels()
        self.cork_weird_8 = False
        self._set_neutral_all_channels()
        sleep(1)

        self._move_forward = True  # Move forward
        sleep(0.5)
        self._set_neutral_all_channels()
        self._move_forward = False
        self._set_neutral_all_channels()
        sleep(0.5)

        # Flash lights [0:35]
        self.flash_light = True  # Activate flashlights
        sleep(3)  # Flashlights on for 3 seconds
        self._set_neutral_all_channels()
        self.flash_light = False
        self._set_neutral_all_channels()
        sleep(1)

        self.zigzag = True  # Perform zigzag maneuver
        sleep(1)  # Assuming zigzag maneuver takes 1 second
        self._set_neutral_all_channels()
        self.zigzag = False
        self._set_neutral_all_channels()
        sleep(1)

        self.spiral_up = True  # Spiral up
        sleep(3)  # Assuming spiral up for 3 seconds
        self._set_neutral_all_channels()
        self.spiral_up = False
        self._set_neutral_all_channels()
        sleep(0.5)

        # Pen-pineapple-apple-pen [0:43]
        self.flip = True  # Flip 360 degrees
        sleep(4)  # Assuming flip maneuver takes 4 seconds
        self._set_neutral_all_channels()
        self.flip = False
        self._set_neutral_all_channels()
        sleep(1)

        self._move_right = True  # Move right
        sleep(1)  # Assuming move right for 1 second
        self._set_neutral_all_channels()
        self._move_right = False
        self._set_neutral_all_channels()
        sleep(1)

        # Spiral up and flashing lights [0:47]
        self.spiral_up = True  # Spiral up
        sleep(3)  # Assuming spiral up for 3 seconds
        self._set_neutral_all_channels()
        self.spiral_up = False
        self._set_neutral_all_channels()
        sleep(1)

        self.flash_light = True  # Activate flashlights
        sleep(4)  # Flashlights on for 4 seconds
        self._set_neutral_all_channels()
        self.flash_light = False
        self._set_neutral_all_channels()
        sleep(1)

        # End of routine with final lights off
        self.lightsoff = True  # Turn lights off
        sleep(1)  # Lights off for 1 second
        self._set_neutral_all_channels()
        self.lightsoff = False
        sleep(1)  # Light off duration
    
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
