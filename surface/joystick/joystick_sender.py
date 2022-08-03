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
import pygame
import sys
import argparse
import socket
import os
from std_msgs.msg import Bool

from wurov.msg import surface_command, io_request
from geometry_msgs.msg import Accel

class joystick_sender:
    def __init__(self):
        rospy.init_node('joystick_sender_'+socket.gethostname(), anonymous=False) 

        self.publisher = rospy.Publisher('surface_command', surface_command, queue_size=3)
        self.light_publisher = rospy.Publisher('light_control', Bool, queue_size=3)


        parser = argparse.ArgumentParser("Find a plugged in joystick and send it over /surface_command.")
        parser.add_argument('--config_name', type=str, help='Set the name of the file we should use as a config (from '
                                                            'within the config directory)', required=True)
        self.args = parser.parse_args(rospy.myargv()[1:])
        namespace = f"/{rospy.get_name()}/Controllers/{self.args.config_name}"

        self.controllerConfig = rospy.get_param(namespace)

        self.already_sent_zero = True  # Set to true so that we aren't trying to set anything to zero on startup
        self.last_sent = ""
        self.servo_in_position = True
        self.stepper_already_moving = True
        self.thruster_already_killed = True
        self.thruster_already_unkilled = True
        self.joystick = None

        # We'll use this to try not to connect to the joystick too quickly.
        rate = rospy.Rate(5)

        #Set pygame to headless
        os.environ["SDL_VIDEODRIVER"] = "dummy"
        pygame.display.init()

        try: #Try to init joystick
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

        # Now that we're not using the rate to slow down our joystick connection, let's bring it to something we'll use.
        rospy.Timer(rospy.Duration(0.1), self.update)
        rospy.spin()

    def update(self, data):
            lastmsg = surface_command()

            pygame.event.get()
            horizontal_axis = self.joystick.get_axis(self.controllerConfig["translationX_axis"])   # Vertical: -1 is full forward, 1 is full back 
            vertical_axis = self.joystick.get_axis(self.controllerConfig["translationY_axis"])  # Horizontal: -1 is full left, 1 is full right
            twist_axis = self.joystick.get_axis(self.controllerConfig["translationZ_axis"])  # Twist: -1 is full counter-clockwise, 1 is clockwise
            
            if self.controllerConfig["depth_axis"]["inputCount"] == 1:
                  depth_axis = self.joystick.get_axis(self.controllerConfig["depth_axis"]["inputOne_Axis"])  # Lever: 1 is full down, -1 is full up
            elif self.controllerConfig["depth_axis"]["inputCount"] == 2:
                depthAxisOne = -1 * self.joystick.get_axis((self.controllerConfig["depth_axis"]["inputOne_Axis"]))
                depthAxisTwo = self.joystick.get_axis((self.controllerConfig["depth_axis"]["inputTwo_Axis"]))

                if depthAxisOne > 0:
                    depthAxisOne = 0
                if depthAxisTwo < 0:
                    depthAxisTwo = 0

                depth_axis = depthAxisOne + (depthAxisTwo)
            else:
                rospy.loginfo("Depth config set wrong")
                quit()

            self.msg = surface_command()
            self.msg .header.stamp = rospy.Time.now()
            self.msg.desired_trajectory.linear.x = -1 * vertical_axis
            self.msg.desired_trajectory.linear.y = horizontal_axis
            self.msg.desired_trajectory.linear.z = depth_axis # Flipped: forward is negative, that's dumb
            self.msg.desired_trajectory.angular.z = twist_axis
            
            msg = self.handle_peripherals(self.joystick, self.msg)
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

        return msg1.desired_trajectory.angular != msg2.desired_trajectory.angular or \
            msg1.desired_trajectory.linear != msg2.desired_trajectory.linear or \
            msg1.io_requests != msg2.io_requests

    def hat_to_val(self, a, b):
        if a == 0:
            if b == 0:
                return None
            if b == 1:
                return "top_front"
            if b == -1:
                return "top_back"
        if a == 1:
            if b == 0:
                return "top_right"
            if b == 1:
                return "front_right"
            if b == -1:
                return "back_right"
        if a == -1:
            if b == 0:
                return "top_left"
            if b == 1:
                return "front_left"
            if b == -1:
                return "back_left"


    def handle_peripherals(self, joystick, msg):
        hat = joystick.get_hat(0)
        hat = self.hat_to_val(hat[0], hat[1])
        io_request_ = io_request()

        if hat is None:
            if not self.already_sent_zero:
                io_request_.executor = "individual_thruster_control"
                io_request_.string = self.last_sent
                self.already_sent_zero = True
                msg.io_requests += (io_request_,)
        else:
            io_request_.executor = "individual_thruster_control"
            io_request_.string = hat
            io_request_.float = 0.75
            self.last_sent = hat
            self.already_sent_zero = False
            msg.io_requests += (io_request_,)

        if joystick.get_button(self.controllerConfig["safetyButton"]):  # Safety trigger: Do not Send trajectory data if this trigger is held.
            msg.desired_trajectory = Accel()

        if joystick.get_button(self.controllerConfig["lightOn"]):  # Safety trigger: Do not Send trajectory data if this trigger is held.
            self.light_publisher.publish(True)

        if joystick.get_button(self.controllerConfig["lightOff"]):  # Safety trigger: Do not Send trajectory data if this trigger is held.
            self.light_publisher.publish(False)

        if not joystick.get_button(self.controllerConfig["boostMode"]):  # 'Boost mode': If this button is pressed, multiply trajectory by 2
            # We implement this by always cutting by 2, and then when the button is pressed, not cutting in half.
            msg.desired_trajectory.linear.x = msg.desired_trajectory.linear.x / 2
            msg.desired_trajectory.linear.y = msg.desired_trajectory.linear.y / 2
            msg.desired_trajectory.linear.z = msg.desired_trajectory.linear.z / 2
            msg.desired_trajectory.angular.x = msg.desired_trajectory.angular.x / 2
            msg.desired_trajectory.angular.y = msg.desired_trajectory.angular.y / 2
            msg.desired_trajectory.angular.z = msg.desired_trajectory.angular.z / 2

        if joystick.get_button(self.controllerConfig["killThrusters"]):  # Kill thrusters button
            if not self.thruster_already_killed:
                io_request_ = io_request()
                io_request_.executor = "kill_thruster"
                io_request_.string = "front_left"
                msg.io_requests += (io_request_,)

                io_request_ = io_request()
                io_request_.executor = "kill_thruster"
                io_request_.string = "front_right"
                msg.io_requests += (io_request_,)

                io_request_ = io_request()
                io_request_.executor = "kill_thruster"
                io_request_.string = "top_front"
                msg.io_requests += (io_request_,)

                self.thruster_already_killed = True
        elif self.thruster_already_killed:  # Make sure the button is released before we send another stream of kill stuff
            self.thruster_already_killed = False

        if joystick.get_button(self.controllerConfig["unkillThrusters"]):  # Un-kill thrusters button
            if not self.thruster_already_unkilled:
                io_request_ = io_request()
                io_request_.executor = "unkill_thruster"
                io_request_.string = "front_left"
                msg.io_requests += (io_request_,)

                io_request_ = io_request()
                io_request_.executor = "unkill_thruster"
                io_request_.string = "front_right"
                msg.io_requests += (io_request_,)

                io_request_ = io_request()
                io_request_.executor = "unkill_thruster"
                io_request_.string = "top_front"
                msg.io_requests += (io_request_,)

                self.thruster_already_unkilled = True
        elif self.thruster_already_unkilled:  # Make sure the button is released before we send another stream of un-kill stuff
            self.thruster_already_unkilled = False

        return msg  # If we wanted to do something with button presses, we could mess around with that sort of thing here.



if __name__ == '__main__':
    joystick_sender()
