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

import rospy
import argparse
import time
from wurov.msg import thruster_sensor, thrustermove, arbitrary_pca_commands
from threading import Thread
from board import SCL, SDA
import busio
# The thruster_dictionary takes sensible thruster names and turns them into PCA channels.
# We default to -1 as a flag value.
thruster_dictionary = {
    'top_front': -1,
    'top_back': -1,
    'top_left': -1,
    'top_right': -1,
    'front_left': -1,
    'front_right': -1,
    'back_left': -1,
    'back_right': -1
}

dead_thrusters = []

MAX_ATTEMPT_COUNT = 5
MIN_PCA_INT_VAL = None
MAX_PCA_INT_VAL = None
PCA_FREQ_VAL = 400
PCA_CONTROL_LOCK = False

try:
    from adafruit_pca9685 import PCA9685
except:
    rospy.logerr("Failed to import Adafruit PCA library.")

pca = None
try:
    i2c_bus = busio.I2C(SCL, SDA)
    pca = PCA9685(i2c_bus)
except:
    rospy.logerr("Failed to initialize PCA.")


# Lock the PCA so that we can guarantee only one function is telling it what to do at a time.
# You *MUST* call release_pca_control when you're done. Make sure that there is no condition
# or exception that could prevent it from being released, otherwise nothing will be able to move!
def lock_pca_control():
    global PCA_CONTROL_LOCK
    PCA_CONTROL_LOCK = True


def release_pca_control():
    global PCA_CONTROL_LOCK
    PCA_CONTROL_LOCK = False


def scale(value):
    return int((value * (MAX_PCA_INT_VAL - MIN_PCA_INT_VAL)) + MIN_PCA_INT_VAL)


def stop_thrusters():
    """
    This runs after the rospy.spin() ends. For safety's sake, turn all the thrusters down to .5, and then to actually
    zero pulsewidth: This de-initializes the ESC, preventing it from accidentally being set to anything else later.
    """
    if pca is None:
        rospy.loginfo("[simulating PCA]: Stopping thrusters.")
        return

    for thruster in thruster_dictionary.keys():
        try:
            if thruster_dictionary[thruster] == -1:  # Flag value: there is no -1 pca channel, skip this thruster
                rospy.logdebug("Did not move due to no channel specification: " + thruster)
            else:
                rospy.logdebug(thruster + " " + str(thruster_dictionary[thruster]))
                pca.channels[thruster_dictionary[thruster]].duty_cycle = scale(.5)  # Stop moving
                time.sleep(.5)  # If you go too fast between IO requests it can throw an error.
                pca.channels[thruster_dictionary[thruster]].duty_cycle =  0  # Kill the channel
                time.sleep(.5)
        except Exception as e:
            rospy.logerr(
                "Thruster stopping failed: Attempting " + thruster + " on " + str(thruster_dictionary[thruster]))
            rospy.logerr("Error cause: " + str(e))

    rospy.loginfo("Stopped thrusters.")


def init_thrusters(init_sequence):
    """
    Initialize the thrusters.
    """
    if pca is None:
        rospy.loginfo("[simulating PCA]: Initializing thrusters.")
        return

    # Run through the thrusters we've got, and if it's a valid thruster, try to initialize it.
    for thruster in thruster_dictionary.keys():
        try:
            if thruster_dictionary[thruster] == -1:  # Flag value: there is no -1 pca channel, skip this thruster
                rospy.logdebug("Did not move due to no channel specification: " + thruster)
            else:
                rospy.logdebug(thruster + " " + str(thruster_dictionary[thruster]))
                for point in init_sequence:
                    persistent_pca(thruster_dictionary[thruster], scale(point))
                    time.sleep(1)
                persistent_pca(thruster_dictionary[thruster], scale(0.5))
                time.sleep(0.25)  # Make sure we're not spinning anymore
        except Exception as e:
            rospy.logerr(
                "Thruster initialization failed: Attempting " + thruster + " on " + str(thruster_dictionary[thruster]))
            rospy.logerr("Error cause: " + str(e))

    rospy.loginfo("Initialized thrusters!")


def move_callback(data):
    """
    This is what runs when a new message comes in from our thruster_commands subscriber.
    """
    if pca is None:
        rospy.loginfo("[simulating PCA]: Move callback entered. msg:\n" + str(data))
        return

    # Convert data into a nicer dictionary
    thruster_values = {
        'top_front': data.top_front,
        'top_back': data.top_back,
        'top_left': data.top_left,
        'top_right': data.top_right,
        'front_left': data.front_left,
        'front_right': data.front_right,
        'back_left': data.back_left,
        'back_right': data.back_right
    }

    # If a thruster has been labeled 'dead', override whatever the other math wants to do to it and don't let it move.
    for thruster in dead_thrusters:
        rospy.logwarn("Thruster "+thruster+" is dead, so we're not moving it.")
        thruster_values[thruster] = 0.5

    # This callback sends a LOT of PCA commands, which can drown out PCA commands from other places. If one of the
    # other places 'locks' the PCA, they're saying that only they can use it for right now. This has to be done quickly
    # though, when the PCA is locked by another callback we're not able to move anything here.
    # Note that this will incidentally come up if you're calling persistent_pca as a result of joystick interaction
    # (i.e., pressing a button), and that's likely OK. Just make sure release_pca_control() is definitely, for sure,
    # 100% of the time, always called when lock_pca_control is called. If you've done that, there's nothing to worry
    # about with this coming up (happens sometimes if you're moving the joystick while a button is first pressed).
    if PCA_CONTROL_LOCK:
        rospy.logdebug("We were going to move, but PCA control is currently locked. Has it been released properly?")
        return

    # Run through all the thruster options we've got
    for thruster in thruster_dictionary.keys():
        try:
            if thruster_dictionary[thruster] == -1:  # Flag value: there is no -1 pca channel, skip this thruster
                rospy.logdebug("Did not move due to no channel specification: " + thruster)
            else:
                rospy.logdebug(
                    thruster + " " + str(thruster_dictionary[thruster]) + " " + str(thruster_values[thruster]))
                pca.channels[thruster_dictionary[thruster]].duty_cycle = scale(thruster_values[thruster])
        except Exception as e:
            # It's possible for the rate of IO requests to be such that the PCA freaks out
            # because we're a bit too quick. We'll catch them and move on, it'll be OK.
            rospy.logdebug("Thruster movement failed: Attempting " + thruster + " at " +
                           str(scale(thruster_values[thruster])) + " (" +
                           str(thruster_values[thruster]) + ") on " +
                           str(thruster_dictionary[thruster]))
            rospy.logdebug("Error cause: " + str(e))


def sensor_callback(data):
    """
    If there are sensors on your thrusters, here's where you can deal with that.
    """
    if data.estop:
        rospy.logerr("estop triggered on " + data.thruster)
        kill_thruster(data.thruster)


def persistent_pca(channel, pwm):
    """
    The usual way we set the PCA does not care if it succeeds or fails, because we know another command is just
    around the corner. However, this is used for things like setting PWM for a servo, which ideally we're only
    sending once. If it fails the same way, that would be bad, so we need a more persistent alternative.
    """
    keep_trying = True
    attempt_count = 0
    if channel is None:
        rospy.logerr("You told the persistent_pca function to set a thruster with a value None!")
        return

    while keep_trying:
        try:
            # Lock the PCA to make sure that only we're using it
            lock_pca_control()
            rospy.logdebug("[persistent_pca] Setting PCA channel " + str(channel) +
                           " to " + str(pwm) + " (attempt #" + str(attempt_count) + ")")
            pca.channels[channel].duty_cycle = pwm

            # If we got here, the PCA didn't freak out. It'll do that sometimes if we request things too back-to-back.
            keep_trying = False
        except Exception as e:
            attempt_count += 1
            time.sleep(0.01)
            if attempt_count > MAX_ATTEMPT_COUNT:
                rospy.logwarn("After " +
                              str(attempt_count) + " attempts, we failed to set the PCA to " +
                              str(pwm) + " (channel " +
                              str(channel) + ")")
                rospy.logerr(e)  # We're assuming that it failed for the same reason each time.
                keep_trying = False

    release_pca_control()


def kill_thruster(thruster):
    dead_thrusters.append(thruster)
    persistent_pca(thruster_dictionary[thruster], scale(0.5))  # Stop the thruster


def unkill_thruster(thruster):
    if thruster in dead_thrusters:
        dead_thrusters.remove(thruster)
        rospy.logwarn("[unkill thruster] Removed "+thruster+" to produce "+str(dead_thrusters))
    else:
        rospy.logwarn("Unkilling thruster failed: You tried unkilling "+thruster+", which is not in "+ str(dead_thrusters))


def set_pwm_after_time(channel, delay, pwm):
    time.sleep(delay)
    persistent_pca(channel, pwm)


def arbitrary_pca_callback(data):
    """
    We can use the PCA for more than just the thrusters: the additional channels make it useful for servos, steppers,
    etc. This callback allows us to take advantage of the PCA for that sort of thing.
    """
    if pca is None:
        rospy.loginfo("[simulating PCA]: arbitrary pca callback entered. msg:\n" + str(data))

        if data.thruster not in thruster_dictionary and data.thruster != '':
            rospy.logerr("[simulating PCA]: You tried specifying a non-existent thruster [" + data.thruster + "]")
        elif data.unkill_thruster and data.thruster not in dead_thrusters:
            rospy.logerr("[simulating PCA]: You tried unkilling a thruster that isn't dead!")
        return

    runcount = 0  # We'll use this to check that the message gave only one command. More than that doesn't make sense.
    if data.set_thruster:
        try:
            if data.thruster == "all":
                for thruster in thruster_dictionary.keys():
                    persistent_pca(thruster_dictionary[thruster], scale(data.pwm))
            elif data.thruster != '':
                if thruster_dictionary[data.thruster] is not None and thruster_dictionary[data.thruster] != -1:
                    persistent_pca(thruster_dictionary[data.thruster], scale(data.pwm))
                else:
                    rospy.logerr("Your arbitrary PCA command requested a thruster operation, but set no thruster name!")
            else:
                rospy.logwarn("You've tried to specify a thruster with no channel association!")
            runcount += 1
        except Exception as e:
            rospy.logerr("Calling arbitrary_pca_callback set_thruster on thruster: "+data.thruster+" failed.\n"+str(e))

    if data.set_channel_pwm:
        persistent_pca(data.channel, int(data.pwm * 4096))
        runcount += 1

    if data.set_channel_pwm_send_count:
        persistent_pca(data.channel, int(0.5 * 4096))  # 50% pulsewidth

        # By running this in a thread, we're non-blocking while making sure the PWM is set back to 0 at the right time
        Thread(target=set_pwm_after_time, args=(data.channel, data.count/400, 0)).start()
        runcount += 1

    if data.kill_thruster:
        kill_thruster(data.thruster)
        runcount += 1

    if data.unkill_thruster:
        unkill_thruster(data.thruster)
        runcount += 1

    if runcount != 1:
        rospy.logwarn(
            "Your arbitrary PCA command specified " + str(runcount) + " operations. You are using this message " +
            "incorrectly, set precisely one operation to perform!")


def listener(arguments):
    """
    Listen to thruster commands and run them, assuming it's ours.
    """
    # Run listener nodes, with the option of happening simultaneously.
    rospy.init_node('thrusters', anonymous=True)

    # Set up subscribers to receive thruster movement commands, to receive thruster sensor inputs, and to
    # receive arbitrary PCA commands, respectively.
    rospy.Subscriber('thruster_commands', thrustermove, move_callback)
    rospy.Subscriber('thruster_sensors', thruster_sensor, sensor_callback)
    rospy.Subscriber('arbitrary_pca_commands', arbitrary_pca_commands, arbitrary_pca_callback)

    if pca is None:
        # Defaulting to a simulation mode (like we do here) is a good default to allow the usage of only one system.
        # However, we need to make sure the user knows what's going on so they don't get too tripped up.
        rospy.logerr("PCA failed to initialize. We're going to assume that's because you're not running on the"
                     " right hardware, and we'll run in simulation instead.")

    # We don't want to initialize until we've got the ability to control the system. This will block until
    # we know something is publishing to the surface_command topic (i.e., a joystick is hooked into the system)
    # We perform this if the --no_wait command line parameter has not been set, making this the default behavior.
    if "no_wait" in arguments:
        pass  # skip the wait, go right to initialization
    else:
        # Loop until we see surface_command being published to
        while ['/surface_command', 'wurov/surface_command'] not in rospy.get_published_topics():
            rospy.loginfo("'surface_command' is still not being published. We'll keep waiting until it's available.")
            time.sleep(2)

    # The default initialization sequence is the BlueESC sequence: mid, a little high, mid.
    init_sequence = [0.5, 0.75]
    if arguments.init_sequence is not None:
        init_sequence = [int(item) for item in arguments.init_sequence.split(' ')]

    # Initialize the thrusters before we attempt to receive any movement commands.
    init_thrusters(init_sequence)

    # We've got our subscribers set up, arguments parsed, and thrusters initialized.
    # Now we run until told to stop.
    rospy.spin()

    # If we're here, the process has been killed
    rospy.loginfo("Told to shut down. Stopping movement and setting channels to 0...")
    stop_thrusters()


if __name__ == '__main__':
    # 'parser' allows us to receive and parse command line arguments.
    parser = argparse.ArgumentParser("Create a ROS interface for the PCA9685 hardware to control thrusters and other "
                                     "motors.")
    parser.add_argument('min_pca_int_value', type=int,
                        help='Integer value that represents the lowest value to be passed to the PCA.')
    parser.add_argument('max_pca_int_value', type=int,
                        help='Integer value that represents the highest value to be passed to the PCA.')
    parser.add_argument('--frequency', type=int,
                        help='The value to be passed to the set_pwm_freq() function. Note that this should produce a '
                             'frequency of 400, so scale accordingly.',
                        default=400)
    parser.add_argument('--no_wait', type=bool,
                        help='By default, this node will not initialize the thrusters until after we detect a '
                             'joystick connecting. If this flag is set, the node will not wait and will instead '
                             'initialize thrusters immediately.',
                        default=argparse.SUPPRESS)
    parser.add_argument('--init-sequence',
                        help="ESC's generally have a specific startup sequence. The default value of '0.5 0.75' "
                             "can be overridden here by passing in space-seperated values ranging from 0 (your "
                             "minimum) to 1 (your maximum).")
    parser.add_argument('--top_front', type=int, help="Set the PCA channel for the top front thruster.")
    parser.add_argument('--top_right', type=int, help="Set the PCA channel for the top right thruster.")
    parser.add_argument('--top_back', type=int, help="Set the PCA channel for the top back thruster.")
    parser.add_argument('--top_left', type=int, help="Set the PCA channel for the top left thruster.")
    parser.add_argument('--front_right', type=int, help="Set the PCA channel for the front right thruster.")
    parser.add_argument('--front_left', type=int, help="Set the PCA channel for the front left thruster.")
    parser.add_argument('--back_right', type=int, help="Set the PCA channel for the back right thruster.")
    parser.add_argument('--back_left', type=int, help="Set the PCA channel for the back left thruster.")

    # ROS likes to slap some other command line args if we're running from roslaunch/launchfile. This allows us to
    # strip those away so we're just looking at what we've passed in.
    args = parser.parse_args(rospy.myargv()[1:])

    MAX_PCA_INT_VAL = args.max_pca_int_value
    MIN_PCA_INT_VAL = args.min_pca_int_value

    # Set the values to map thrusters to PCA channels.
    if args.top_front is not None:
        thruster_dictionary['top_front'] = args.top_front
    if args.top_right is not None:
        thruster_dictionary['top_right'] = args.top_right
    if args.top_back is not None:
        thruster_dictionary['top_back'] = args.top_back
    if args.top_left is not None:
        thruster_dictionary['top_left'] = args.top_left
    if args.front_right is not None:
        thruster_dictionary['front_right'] = args.front_right
    if args.front_left is not None:
        thruster_dictionary['front_left'] = args.front_left
    if args.back_right is not None:
        thruster_dictionary['back_right'] = args.back_right
    if args.back_left is not None:
        thruster_dictionary['back_left'] = args.back_left

    rospy.logwarn(str(thruster_dictionary))

    if pca is None:
        rospy.logwarn("[simulated PCA]: Setting frequency.")
    elif args.frequency is not None:
        pca.frequency = args.frequency

    if MIN_PCA_INT_VAL <= MAX_PCA_INT_VAL:
        rospy.logerr("Your max PCA value <= your min PCA value. Swapping, but you've configured stuff wrong so it'll "
                     "probably break elsewhere, too.")
        tmp = MAX_PCA_INT_VAL
        MAX_PCA_INT_VAL = MIN_PCA_INT_VAL
        MIN_PCA_INT_VAL = tmp

    listener(args)
