#!/usr/bin/env python3

"""

 This file is part of Enbarr.

    Enbarr is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Enbarr is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Enbarr.  If not, see <https://www.gnu.org/licenses/>.

"""

import os
import rospy
import rospkg
import pygame
import sys
import argparse
import socket
from wurov.msg import surface_command, io_request
import time

class imu_data:
    def __init__(self):
        self.publisher = rospy.Publisher('surface_command', surface_command, queue_size=3)
        rospy.init_node('joystick_sender_'+socket.gethostname(), anonymous=False)  # Be effectively anonymous by naming the node after the hostname

        self.joystick = None

        # We'll use this to try not to connect to the joystick too quickly.
        rate = rospy.Rate(1)

        os.environ["SDL_VIDEODRIVER"] = "dummy"
        pygame.display.init()

        try:
            pygame.joystick.init()
            while pygame.joystick.get_count() == 0:
                rospy.logerr("No joystick connected!")
                #pygame.joystick.quit()
                pygame.joystick.init()
                rate.sleep()
            rospy.loginfo("Found a joystick to use.")
            pygame.init()
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
        except pygame.error:
            rospy.logerr("Failed to initialize joystick!")
            sys.exit(1)

        parser = argparse.ArgumentParser("Find a plugged in joystick and send it over /surface_command.")
        parser.add_argument('--config_name', type=str, help='Set the name of the file we should use as a config (from '
                                                            'within the config directory)')
        self.args = parser.parse_args(rospy.myargv()[1:])

        # roslaunch/rosrun executes this from the wrong directory, preventing us from calling the config import.
        # By updating our python path via sys, we're able to tell it where to find this stuff.
        # TODO: Make this conditional on a command line parameter.
        rospkg = rospkg.RosPack()
        sys.path.append(rospkg.get_path('wurov'))
        import config

        # Now that we're not using the rate to slow down our joystick connection, let's bring it to something we'll use.
        rospy.Timer(rospy.Rate(10), self.update)
        rospy.spin()

    def update(self):
            lastmsg = surface_command()

            pygame.event.get()
            horizontal_axis = self.joystick.get_axis(0)  # Horizontal: -1 is full left, 1 is full right
            vertical_axis = self.joystick.get_axis(1)  # Vertical: -1 is full forward, 1 is full back
            twist_axis = self.joystick.get_axis(2)  # Twist: -1 is full counter-clockwise, 1 is clockwise
            lever_axis = self.joystick.get_axis(3)  # Lever: 1 is full down, -1 is full up
            self.msg = surface_command()
            self.msg.desired_trajectory.translation.x = -1 * horizontal_axis
            self.msg.desired_trajectory.translation.y = vertical_axis  
            self.msg.desired_trajectory.translation.z = -1 * lever_axis # Flipped: forward is negative, that's dumb
            self.msg.desired_trajectory.orientation.yaw = -1 * twist_axis
            
            msg = self.config.simulate_peripherals.handle_peripherals(self.joystick, msg)
            print(msg)
            if self.different_msg(lastmsg, msg):
                self.publisher.publish(msg)
                lastmsg = msg

    def hat_to_val(self, a, b):
        if a == 0:
            if b == 0:
                return None
            if b == 1:
                return "top_front"
            if b == -1:
                return "top_left"
        if a == 1:
            if b == 0:
                return "top_right"
            if b == 1:
                return "front_right"
            if b == -1:
                return "front_left"
        if a == -1:
            if b == 0:
                return "top_back"
            if b == 1:
                return "back_left"
            if b == -1:
                return "back_right"


    def handle_peripherals(self, joystick_, msg_):
        hat = joystick_.get_hat(0)
        hat = self.hat_to_val(hat[0], hat[1])

        io_request_ = io_request()
        io_request_.executor = "individual_thruster_control"

        if hat is not None:
            io_request_.float = 0.75
            io_request_.string = hat
        else:
            io_request_.float = 0.5
            io_request_.string = "all"
        self.msg.io_requests += (io_request_,)

        return msg_  # If we wanted to do something with button presses, we could mess around with that sort of thing here.


    def different_msg(self, msg1, msg2):
        if msg1 is None or msg2 is None:
            return True

        return msg1.desired_trajectory.orientation != msg2.desired_trajectory.orientation or \
            msg1.desired_trajectory.translation != msg2.desired_trajectory.translation or \
            msg1.io_requests != msg2.io_requests


if __name__ == '__main__':
    imu_data()