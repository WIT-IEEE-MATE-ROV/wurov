# Enbarr AUV 2.0
Code for the Autonomous Underwater Vehicle produced as a part of the Enbarr project, free for use to create any sort of
underwater ROV or AUV.

## What is this?
This is code to control an underwater ROV/AUV from a surface station. It's designed to be platform-ambivalent (meaning that 
you configure it to your needs and hardware, and let it deal with the rest). As of version 2.0, the plugin/core
structure is no longer implied and is instead the forced standard. Doing it this way allows us to build up a 
repository of different sensors and motor control systems for future teams to build onto, making development quicker for
us now and even quicker for future development.

## File structure
This repository uses the de-facto directory standard found in ROS workspaces.

- `config` contains configuration files, keeping them in one place rather than cluttering up the repository. See the README in that directory for more information.
- `core` contains the core Enbarr code. See the 'core and plugins' section for more.
- `docs` contains more documentation.
- `launch` contains launch files. The files here allow you to run existing configurations in simulation, real life, etc.
- `msg` contains ROS message templates. These are used to pass information from one process to another.
- `plugins` contains Enbarr plugins. See the 'core and plugins' section for more.
- `scripts` contains helper scripts.

## Understanding ROS and how it's being used here
ROS is the [Robot Operating System](https://en.wikipedia.org/wiki/Robot_Operating_System), which is a terrible name
because it's actually a robotics middleware (it's a collection of software and tools to allow you to write code for
robot control). 

ROS allows you to break up your robot code into different independent programs, and ROS will handle the communication
between those different programs. This is useful in our case because we can standardize how those independent
programs talk to each other, and then swap them in and out according to our needs. This allows for complex systems to 
be quickly developed without developers getting in each other's way, and it also allows us to build up a complex 
codebase with pieces that can be easily swapped in and out as they are needed. This individual programs are called
ROS nodes. In the case of this repository, we've categorized the nodes into 'core' and 'plugin' nodes.

To allow this to happen, we use launch files to start a combination of core and plugin ros nodes. Launch files are
files that tell ROS about a large number of nodes that you'd like to launch at one time. This allows for a 
complex setup to be put together once, and then used repeatedly without too much of a hassle (you can use the roslaunch
command to run a launch file, so now running a complex system of 10+ nodes is a single command).

To pass information from program to program, we use messages and topics. A message is a way to describe to ROS an object
that we want to be able to pass from program to program. Examples of ours can be found in the `msg` directory. Topics
are part of a ['publisher-subscriber model'](https://en.wikipedia.org/wiki/Publish%E2%80%93subscribe_pattern): nodes can
publish information to a topic, which makes it available to other nodes that are subscribed to a topic. A topic has a name
so that we can tell them apart, and has a message type so that everyone dealing with it knows what sort of data to expect.

With this system, we can have a bunch of interchangeable nodes that can be chained together to create systems that do
some really complex things, but are quick to assemble, understand, develop, and debug. For more reading on ROS, check
out the [introduction](https://wiki.ros.org/ROS/Introduction) on the ros wiki, or for more reading, the
[concepts](https://wiki.ros.org/ROS/Concepts) page. ROS is well documented on the ROS wiki, and has a large community
answering questions found on [ROS Answers](https://answers.ros.org/questions/).

## Core and Plugins
The code run here falls into two categories: core code, and plugin code. Some things will be universal: we're going to 
want to be able to communicate with a surface station, we're going to want to get sensor data, we're going to want to 
put that through a control loop, etc. Thanks to the way the architecture is set up with our universal messages, this is 
common core code. 

What makes different ROV's and AUV's different from build to build is in the hardware itself, and this is where plugins 
come in. For example, the core code doesn't care about how the thrusters end up at 80% power, it just needs them there. 
It's up to the appropriate plugin to deal with how to make the hardware do the right thing, and it's up to you (the 
user/engineer) to make sure that the right plugin is being used on the right hardware. By creating new plugins for new 
hardware, we can easily swap in new thrusters/sensors with minimal effort, allowing for greater adaptability and focus
 on improving core code or challenge-specific code.
 
## A simple illustrative example
The most basic case can be found in simulation by running `roslaunch auv simulate.launch`. This should bring up a system
that can be viewed using `rqt_graph`, a simplified version of which is shown here:

![A graph of the simulate launch file](docs/simple_rosgraph.png)

Note that the circular pieces of this graph are ROS nodes (single programs running independently), while the arrows 
represent topics (publisher pointing to subscriber, with the text being the topic name). 

We'll work from left to right to show what each of these nodes are doing and why they're necessary. 

##### /imu/data_raw
`imu/data_raw` is the Nine Degrees of Freedom sensor. Examples include the [LSM9DS1](https://www.adafruit.com/product/3387)
or the [NXP](https://www.adafruit.com/product/3463), both of which provide magnetometer, gyroscope, and accelerometer
data. This data is useful for things like control loops or gathering position data.

`/imu/data_raw`, in this case, is `simulate_imu_data.py` found within `plugins/sensors`. It's a plugin because it deals
directly with hardware (in this case simulated), so it may need to be swapped out for another sensor node. To make a 
new `imu/data_raw` node, the easiest way is to make a copy of the `simulate_imu_data.py` file and make it read your sensor
instead of generating random values. The important part is that it publishes the right message to `/imu/data`.

##### /filtering
So you've got your position/orientation sensor values. However, the nature of hardware is that it's not going to be
perfect data. To correct for noise, some sort of filtering needs to be done. Thanks to all nine degree of freedom 
sensors being published on a standard topic, we can use the same filtering math for all of them. The current filtering
that has been implemented is a [Kalman filter](https://en.wikipedia.org/wiki/Kalman_filter), but because it's a ROS
node, it could be swapped out for something like a [Madgewick filter](https://stackoverflow.com/questions/23599256/implementing-madgwick-imu-algorithm).

This is a part of the core because it can be used regardless of what hardware is being used. It can be found
 in `core/filtering/kalman.py`. All a filter needs to do is take in `/imu/data_raw` and spit out processed values to
 `/imu/data`. 
 
##### /command_receiver
The command receiver receives commands that the operator would like the ROV to run. These commands will typically come 
from a surface station. However, thanks to a topic being able to have multiple publishers, it would be easy to set up 
your ROV to also be an AUV by having another node also send commands to `/command_receiver`. 

The command receiver being run here is looking for commands on the topic `surface_command`, but it could be swapped out 
for a node that looks for commands over a socket, for example. This is a part of core because it can be used regardless
of hardware, so it can be found in `core/control/command_receiver.py`.

When a command is received, it is parsed and published in one of two places: either it will be published to 
`/io_request`, which allows for interaction with IO devices (such as GPIO or a servo), or it will be published to
`/trajectory_request`, which allows for control of thrusters.

##### /control_loop
A control loop is a tool in [control theory](https://en.wikipedia.org/wiki/Control_theory) that allows us to take in sensor
 data and a goal, and use them together to produce more accurate movement. In the simulation, a PI controller (which 
 is a variation of a [PID controller](https://en.wikipedia.org/wiki/PID_controller)) is being used as our control loop.
This node takes in the information about where we want to be and where we actually are, and does the math to figure out 
how to make that happen.

Note that the 'where we actually are' data needs to be interpreted by the control loop node slightly. When getting the
'where we want to be' data, the user is actually giving a velocity, not a position: moving a joystick isn't saying
"Go to the point (x:5, y:10, z:15)", it's saying "Go forward fast and turn to the right a little". For this reason,
the filtered 9-DoF data needs to be used with that in mind. The corrected trajectories are then published to 
`/trajectory_corrected`.

Because a control loop can be used regardless of what hardware is being run, this is found in the core. The control loop
being used in this simulation can be found in `core/control_loop/control_loop_pi.py`.

##### /thruster_converter
At this stage, we have a goal that has been corrected with filtered data. However, that goal is represented in 
translation/orientation data, which isn't helpful when it comes to actually running anything. This is where
`thruster_converter` comes into play: it takes the goal data and turns it into values to be executed by individual
thrusters. Different converters can be implemented for different layouts.

The converted result will have a value for each thruster (top_right, top_left, etc). The values will be between 0 and
1. 0 represents the lower extreme and 1 represents the upper extreme, and so .5 will be no movement on bi-directional
thrusters. This is done instead of the -1 to 1 range used elsewhere because it is easier to use when actually controlling
the thrusters, and it allows for the usage of single-direction thrusters in the future.

At the time of writing, only vector drive thruster layouts have ever been used. For that reason, the only trajectory
converter is `vector_trajectory_converter`, found in `core/trajectory_converter/vector_trajectory_converter.py`.

##### /thruster_control
At this final stage, a movement goal has been received, fed through some filtering and control looping, and converted
into an executable value for each thruster. Now it's time to execute them.

Thruster commands, coming in from `/thruster_commands`, provide a value from -1 to 1 for each thruster. -1 represents 
full speed in reverse, 1 represents full speed forward, and 0 represents no movement. The exact way this is done will
depend on what thruster and ESC is being used, so that's a plugin. The one being used in simulation can be found in 
`plugins/motors/simulate_thruster_control.py`. Many past ROV's have used the 
[PCA9685](https://www.adafruit.com/product/815), so a plugin for that is also provided but incomplete: it will need to
be filled out to control thrusters via the PCA.

In the case of this simulated thruster controller, it's also subscribing to `/thruster_sensors` (not shown because 
nothing is publishing to it). This could allow for maintaining RPM of the thruster, or stopping in the event of an
emergency (such as a tangled thruster).

## Launch files and how to make the most of them here
As mentioned in the [Understanding ROS](#understanding-ros-and-how-its-being-used-here) section, launch files are an essential part of using ROS properly.
This is particularly true of this repository, which assumes that you'll need to swap in and out different nodes for the
sake of coming to a custom configuration. By using launch files to launch nodes that meet your needs, they can be 
treated sort of like configuration files that may change from machine to machine, even though the core codebase will
only grow.
 
 To get a better understanding of launch files and how we're using them, we'll
again use the `simulate.launch` file. The simplified version looks like this:

```
<launch>
    <node name="thruster_control" pkg="auv" type="simulate_thruster_control.py" args="" />
    <node name="imu/data_raw" pkg="auv" type="simulate_imu_data.py" />

    <include file="$(find auv)/launch/core.launch" />
</launch>
```

In this launch file, we're running two ROS nodes: our `thruster_control` node (which is actually 
`simulate_thruster_control.py`), and `imu/data_raw` (which is actually `simulate_imu_data.py`). These are both simulation
nodes, but if you wanted to make a new setup that uses real hardware, you could just change out the `type=` to a node
that is already in this repository and meets your needs. If it doesn't exist, you can make copies of whatever file 
you need and modify it to meet your specific hardware needs. 

This demonstrates the usage of plugins: In potentially as little as two lines, you can completely modify the functionality
of the entire ROV!

You could also add more nodes to deal with different sensors, or different motors to be moved. With
this approach, the software becomes more of a [Systems Engineering](https://en.wikipedia.org/wiki/Systems_engineering)
problem, allowing for robust and complicated systems to be quickly assembled.

For an example, let's make the previous sample more complicated by adding two more nodes:

```
    <node name="manipulator" pkg="auv" type="simulate_stepper.py" args="manipulator" />
    <node name="lights" pkg="auv" type="simulate_pi_gpio.py" args="12" />
```

This produces a new graph:

![Rosgraph, as before, but now with optional plugins](docs/plugins_rosgraph.png)

Notice that both of these new nodes are subscribed to `/command_receiver` via `io_request`. `io_request` is a very
general message, which is why we're able to use the one message to control everything. In this example, we have a node
that deals with stepper motors (`simulate_stepper.py`, found in `plugins/motors/stepper/`). Because we'll be using it for
controlling the manipulator, for ease of use we're naming it 'manipulator'. This particular node takes in an argument,
we're providing 'manipulator' (more on that in a minute).

Similarly, we have a node to control our lights, which is why that's the name of the node. It's controlled by the GPIO
on a Pi, and we have a node for that (`simulate_pi_gpio.py`, found in `plugins/gpio`). Like the stepper plugin, this 
takes in an argument, but this time it's an integer.

When an `io_request` is generated, it's provided with two pieces of information: the executor and the value. The 
executor is a string that indicates what node should be responsible for executing the request held in the value. The way
this is used is up to the person writing the plugin: in the case of the stepper plugin, the argument is used as is, but
in the case of the GPIO plugin, the argument refers to the pin that should be used and generates the executor string
 from that (the node is looking to execute values aimed at gpio_12, in this case). 
 
The second piece of information is the value, which again is dependent on the actual node. There are a handful of data
types in the message that can be set and used. In the case of the stepper, we're using the int32 value to refer to how
many steps should be taken, while in the GPIO plugin we're using the boolean to set a pin high or low. By keeping this
implementation vague, it can be adapted to a larger number of applications.

The final line of the launch file is the `<include`, which points to a file at `launch/core.launch`. This contains:

```
<launch>
    <node name="command_receiver" pkg="auv" type="command_receiver.py" args="" />
    <node name="thruster_converter" pkg="auv" type="vector_trajectory_converter.py" args="" />
    <node name="filtering" pkg="auv" type="kalman.py" />
    <node name="control_loop" pkg="auv" type="control_loop_pi.py" />
</launch>
```

These are all of the nodes that were described as being 'core' functionality rather than plugins. For the sake of ease
of use, they're all in this one launch file that gets called by the `include` of `simulate.launch`, but you could easily
make a new version of this file and treat these nodes like plugins, too. For example, if you want to use a different 
thruster converter, you could make one and swap it in.

Keep in mind that the core.launch exists only for convenience:
it contains values that are probably what you want and need, but if they aren't, there's no need to use it. 

## Setting up for usage

To use this code, you will need the standard ROS Kinetic utilities (catkin, etc) described by the ROS tutorials. Create 
your catkin workspace wherever you wish, and then run 'catkin_make' as described 
[here](https://wiki.ros.org/catkin/Tutorials/create_a_workspace). `cd` on over to the `src` directory (which you should 
have made within your catkin workspace), and `git clone` this repository. You will now be able to run `catkin_make` from
 the root of the catkin workspace to make the package.
The goal of this project is to create a low-cost, entry level AUV (both software platform and hardware) capable of being
 adapted for a wide array of research or hobbyist applications. It's not as capable as some of the larger or more 
 expensive counterparts, but it's low-cost, can be replicated at pretty much any university (or even by an individual, 
 depending on what tools you have around), and it's still capable of collecting data.


It's designed to be pretty easy to use and adapt for your needs. If something is difficult to use, adapt, or understand,
 that's because we've failed to properly document it: please let us know how we can take care of it by creating an issue
 for us to review. If you've made an interesting addition we might be interested in, we're open to pull requests! Check
 out contributing.md.

We've chosen to use Python for our core codebase. However, keep in mind that thanks to ROS, you can use any language ROS
 supports to interact with our system (C++, Python, Java, Lisp...). 

The hardware is Free and Open Source too, and developed using only Free and Open Source tools. The pressure tolerant 
design allows for impressive depths at low cost. Check out the 'hardware' repo for more, but you don't actually need it:
we're running ROS, so you can run your code in simulation before pushing it on to an actual machine.

To get that to happen, you'll need to follow our [setup steps](), and then check out the [default usage]() steps. Once 
you've got that, you can start to play with our [api documentation]() to learn how to read values from sensors or 
instruct the machine to send PWM values, follow a trajectory, and more.

