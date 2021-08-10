Customized Version Instructions for Resilient Planner
--------------------------------------------------------

This is a customized version to use [VID-Fusion](https://github.com/ZJU-FAST-Lab/VID-Fusion) and [External Forces Reselient Planner](https://github.com/ZJU-FAST-Lab/forces_resilient_planner). The planner in the simulator has been tested on ubuntu 18.04 and ROS Melodic.

### 0.1 Run the customed environment

We use the drone "hummingbird" with a vi sensor for testing. After following the instructions of RotorS setting, you can directly run: 

```
source devel/setup.bash && roslaunch rotors_gazebo fast_with_vi_sensor.launch
```

It will also launch a virtual joystick window and you can press keyboard to control the drone. 
(The joy interface is revised as position control rather than altitude control, which is easier for interface.)

The important topics are:

For planner:
- subscribers:
  - `ground truth odom`:  /hummingbird/ground_truth/odometry
  - `depth camera`:  /hummingbird/vi_sensor/camera_depth/depth/disparity

- publishers:
  - `traj cmd`:  /hummingbird/command/trajectory

For VIO:
- subscribers:
  - `imu_topic`:  /hummingbird/ground_truth/imu
  - `image_topic`:  /hummingbird/vi_sensor/left/image_raw
  - `control_topic`:  /hummingbird/thrust

To get camera parameters:

`Camera info`: /hummingbird/vi_sensor/camera_depth/camera/camera_info


### 0.2 Revise camera parameters

It's flexible to change its focal length and the position on the drone.

- internal parameters

In `rotors_description/urdf/component_snippets.xacro`, start with line 626, find a macro `vi_sensor_depth_macro` and revise its camera_plugin to change focal length and distortion.

Tutorials for more instructions: http://gazebosim.org/tutorials/?tut=ros_depth_camera


- external parameters

In `rotors_description/urdf/mav_with_vi_sensor.gazebo`, to revise the origin xyz and rpy.

```xml
  <!-- Mount a VI-sensor in front of the Firefly. -->
  <xacro:vi_sensor_macro
    namespace="${namespace}/vi_sensor"
    parent_link="${namespace}/base_link"
    enable_cameras="true"
    enable_depth="true"
    enable_ground_truth="true">
    <origin xyz="0.1 0.0 0.086" rpy="0.0 0.0 0.0" />
  </xacro:vi_sensor_macro>

```

### 0.3 Adjust external forces

RotorS has two interfaces to add custom wind: the whole wind distribution or wind during time. For conveience, we also add two easier interfaces. 
You can both use `joystick` or `disturbance manager` to add external forces.

- By using joystick/keyboard: you can press y/u h/j n/m to add an external force in three dimension

- By disturbance manager: follow the example in disturbance_manager.cpp to add customed force.


### 0.4 Other Notes

#### Custom the world file

VID-fusion need features to track, so it's better to change the texture in you environment. In `fast_quad.world`, I add several brick box with different texture, and their model can be found in `rotors_gazebo/models`


#### Drone's Mass

Because we use hummingbird for testing, when vi-sensor is placed in the CoG of the quadrotor, the rotors will appear in the camera image.
However if set in front, it will not stable because of the mass of vi-sensor and need to adjust the parameters of position controller.

To easily avoid the issue, we decrease the mass of vi-sensor near zero and calibrate the mass of drone. By testing during drone's hovering, the mass is mass: 0.745319 now. If you directly clone the original verision, you need to revise it in 


#### Drone's Color

In `rotors_description/urdf/hummingbird.xacro` to change the color of the rotors and body of quadrotor.

```xml
  <!-- Instantiate multirotor_base_macro once -->
  <xacro:multirotor_base_macro
    robot_namespace="${namespace}"
    mass="${mass}"
    body_width="${body_width}"
    body_height="${body_height}"
    use_mesh_file="${use_mesh_file}"
    color="Grey"
    mesh_file="${mesh_file}">
    <xacro:insert_block name="body_inertia" />
  </xacro:multirotor_base_macro>
```


------------------------
RotorS
===============

RotorS is a MAV gazebo simulator.
It provides some multirotor models such as the [AscTec Hummingbird](http://www.asctec.de/en/uav-uas-drone-products/asctec-hummingbird/), the [AscTec Pelican](http://www.asctec.de/en/uav-uas-drone-products/asctec-pelican/), or the [AscTec Firefly](http://www.asctec.de/en/uav-uas-drone-products/asctec-firefly/), but the simulator is not limited for the use with these multicopters.

There are simulated sensors coming with the simulator such as an IMU, a generic odometry sensor, and the [VI-Sensor](http://wiki.ros.org/vi_sensor), which can be mounted on the multirotor.

This package also contains some example controllers, basic worlds, a joystick interface, and example launch files.

Below we provide the instructions necessary for getting started. See RotorS' wiki for more instructions and examples (https://github.com/ethz-asl/rotors_simulator/wiki).

If you are using this simulator within the research for your publication, please cite:
```bibtex
@Inbook{Furrer2016,
author="Furrer, Fadri
and Burri, Michael
and Achtelik, Markus
and Siegwart, Roland",
editor="Koubaa, Anis",
chapter="RotorS---A Modular Gazebo MAV Simulator Framework",
title="Robot Operating System (ROS): The Complete Reference (Volume 1)",
year="2016",
publisher="Springer International Publishing",
address="Cham",
pages="595--625",
isbn="978-3-319-26054-9",
doi="10.1007/978-3-319-26054-9_23",
url="http://dx.doi.org/10.1007/978-3-319-26054-9_23"
}
```
Installation Instructions - Ubuntu 16.04 with ROS Kinetic
---------------------------------------------------------
 1. Install and initialize ROS kinetic desktop full, additional ROS packages, catkin-tools, and wstool:

 ```
 $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
 $ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
 $ sudo apt-get update
 $ sudo apt-get install ros-kinetic-desktop-full ros-kinetic-joy ros-kinetic-octomap-ros ros-kinetic-mavlink python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-kinetic-control-toolbox ros-kinetic-mavros
 $ sudo rosdep init
 $ rosdep update
 $ source /opt/ros/kinetic/setup.bash
 ```
 2. If you don't have ROS workspace yet you can do so by

 ```
 $ mkdir -p ~/catkin_ws/src
 $ cd ~/catkin_ws/src
 $ catkin_init_workspace  # initialize your catkin workspace
 $ wstool init
 $ wget https://raw.githubusercontent.com/ethz-asl/rotors_simulator/master/rotors_hil.rosinstall
 $ wstool merge rotors_hil.rosinstall
 $ wstool update
 ```

  > **Note** On OS X you need to install yaml-cpp using Homebrew `brew install yaml-cpp`.

 3. Build your workspace with `python_catkin_tools` (therefore you need `python_catkin_tools`)

   ```
   $ cd ~/catkin_ws/
   $ catkin build
   ```

 4. Add sourcing to your `.bashrc` file

   ```
   $ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
   $ source ~/.bashrc
   ```

Installation Instructions - Ubuntu 14.04 with ROS Indigo
--------------------------------------------------------

 1. Install and initialize ROS indigo desktop full, additional ROS packages, catkin-tools, and wstool:

 ```
 $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
 $ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
 $ sudo apt-get update
 $ sudo apt-get install ros-indigo-desktop-full ros-indigo-joy ros-indigo-octomap-ros python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev
 $ sudo rosdep init
 $ rosdep update
 $ source /opt/ros/indigo/setup.bash
 ```
 2. If you don't have ROS workspace yet you can do so by

 ```
 $ mkdir -p ~/catkin_ws/src
 $ cd ~/catkin_ws/src
 $ catkin_init_workspace  # initialize your catkin workspace
 $ wstool init
 ```
 > **Note** for setups with multiple workspaces please refer to the official documentation at http://docs.ros.org/independent/api/rosinstall/html/ by replacing `rosws` by `wstool`.
 3. Get the simulator and additional dependencies

 ```
 $ cd ~/catkin_ws/src
 $ git clone git@github.com:ethz-asl/rotors_simulator.git
 $ git clone git@github.com:ethz-asl/mav_comm.git
 ```
  > **Note** On OS X you need to install yaml-cpp using Homebrew `brew install yaml-cpp`.

  > **Note** if you want to use `wstool` you can replace the above commands with
    ```
    wstool set --git local_repo_name git@github.com:organization/repo_name.git
    ```
  > **Note** if you want to build and use the `gazebo_mavlink_interface` plugin you have to get MAVROS as an additional dependency from link below. Follow the installation instructions provided there and build all of its packages prior to building the rest of your workspace.
    ```
    https://github.com/mavlink/mavros
    ```
 4. Build your workspace with `python_catkin_tools` (therefore you need `python_catkin_tools`)

   ```
   $ cd ~/catkin_ws/
   $ catkin init  # If you haven't done this before.
   $ catkin build
   ```
   > **Note** if you are getting errors related to "future" package, you may need python future:
    ```
    sudo apt-get install python-pip
    pip install --upgrade pip
    pip install future
    ```

 5. Add sourcing to your `.bashrc` file

   ```
   $ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
   $ source ~/.bashrc
   ```

Basic Usage
-----------

Launch the simulator with a hex-rotor helicopter model, in our case, the AscTec Firefly in a basic world.

```
$ roslaunch rotors_gazebo mav_hovering_example.launch mav_name:=firefly world_name:=basic
```

> **Note** The first run of gazebo might take considerably long, as it will download some models from an online database. Should you receive a timeout error, try running gazebo by itself (e.g. roslaunch gazebo_ros empty_world.launch ) so it has sufficient time to actually download all of the models.

The simulator starts by default in paused mode. To start it you can either
 - use the Gazebo GUI and press the play button
 - or you can send the following service call.

   ```
   $ rosservice call gazebo/unpause_physics
   ```

There are some basic launch files where you can load the different multicopters with additional sensors. They can all be found in `~/catkin_ws/src/rotors_simulator/rotors_gazebo/launch`.

The `world_name` argument looks for a .world file with a corresponding name in `~/catkin_ws/src/rotors_simulator/rotors_gazebo/worlds`. By default, all launch files, with the exception of those that have the world name explicitly included in the file name, use the empty world described in `basic.world`.

### Getting the multicopter to fly

To let the multicopter fly you need to generate thrust with the rotors, this is achieved by sending commands to the multicopter, which make the rotors spin.
There are currently a few ways to send commands to the multicopter, we will show one of them here.
The rest is documented [here](../../wiki) in our Wiki.
We will here also show how to write a stabilizing controller and how you can control the multicopter with a joystick.

#### Send direct motor commands

We will for now just send some constant motor velocities to the multicopter.

```
$ rostopic pub /firefly/command/motor_speed mav_msgs/Actuators '{angular_velocities: [100, 100, 100, 100, 100, 100]}'
```

> **Note** The size of the `motor_speed` array should be equal to the number of motors you have in your model of choice (e.g. 6 in the Firefly model).

You should see (if you unpaused the simulator and you have a multicopter in it), that the rotors start spinning. The thrust generated by these motor velocities is not enough though to let the multicopter take off.
> You can play with the numbers and will realize that the Firefly will take off with motor speeds of about 545 on each rotor. The multicopter is unstable though, since there is no controller running, if you just set the motor speeds.


#### Let the helicopter hover with ground truth odometry

You can let the helicopter hover with ground truth odometry (perfect state estimation), by launching:

```
$ roslaunch rotors_gazebo mav_hovering_example.launch mav_name:=firefly world_name:=basic
```

#### Create an attitude controller

**TODO(ff):** `Write something here.`

#### Usage with a joystick

Connect a USB joystick to your computer and launch the simulation alongside ROS joystick driver and the RotorS joystick node:
```
$ roslaunch rotors_gazebo mav_with_joy.launch mav_name:=firefly world_name:=basic
```

Depending on the type of joystick and the personal preference for operation, you can assign the axis number using the `axis_<roll/pitch/thrust>_` parameter and the axis direction using the `axis_direction_<roll/pitch/thrust>` parameter.

#### Usage with a keyboard

First, perform a one-time setup of virtual keyboard joystick as described here: https://github.com/ethz-asl/rotors_simulator/wiki/Setup-virtual-keyboard-joystick.

Launch the simulation with the keyboard interface using the following launch file:
```
$ roslaunch rotors_gazebo mav_with_keyboard.launch mav_name:=firefly world_name:=basic
```

If everything was setup correctly, an additional GUI should appear with bars indicating the current throttle, roll, pitch, and yaw inputs. While this window is active, the Arrows and W, A, S, D keys will generate virtual joystick inputs, which can then be processed by the RotorS joystick node in the same way as real joystick commands.

Gazebo Version
--------------

At a minimum, Gazebo `v2.x` is required (which is installed by default with ROS Indigo). However, it is **recommended to install at least Gazebo `v5.x`** for full functionlity, as there are the following limitations:

1. `iris.sdf` can only be generated with Gazebo >= `v3.0`, as it requires use of the `gz sdf ...` tool. If this requirement is not met, you will not be able to use the Iris MAV in any of the simulations.
2. The Gazebo plugins `GazeboGeotaggedImagesPlugin`, `LidarPlugin` and the `LiftDragPlugin` all require Gazebo >= `v5.0`, and will not be built if this requirement is not met.
