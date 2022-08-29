# ARTA_docker

This is the usage for using Docker within Ubuntu 20.04/18.04/16.04

## Steps

0. Install docker engine from official website
```
https://docs.docker.com/engine/install/ubuntu/
```
0.1. Using ARTA in real world

In host computer: Install ROS & sudo apt-get install ros-${ROS_DISTRO}-urg-node

1. Create a folder (ie. ~/ARTADockerSetup) 
```
mkdir ~/ARTADockerSetup
```
2. Cd to that folder & download Dockerfile and Makefile into the folder
```
cd ~/ARTADockerSetup
  
wget --no-check-certificate --content-disposition https://github.com/Thomas-Tai/arta_docker/raw/main/Dockerfile
  
wget --no-check-certificate --content-disposition https://github.com/Thomas-Tai/arta_docker/raw/main/Makefile
```
3. Create the Docker image and environment
```
make default
```  
3.1. If Host is using 18.04 , update the rules with following command 
```  
sudo vim /etc/udev/rules.d/96-hokuyo.rules
```    
  Replace from
```    
KERNEL=="ttyACM[0-9]*", ACTION=="add", ATTRS{idVendor}=="15d1", MODE="0666", GROUP="dialout", PROGRAM="/opt/ros/noetic/env.sh rosrun urg_node getID %N q", SYMLINK+="hokuyo_%c"
```    
  To
```    
KERNEL=="ttyACM[0-9]*", ACTION=="add", ATTRS{idVendor}=="15d1", MODE="0666", GROUP="dialout", PROGRAM="/opt/ros/melodic/env.sh rosrun urg_node getID %N q", SYMLINK+="hokuyo_%c"
```    
  3.2. If Host is using 16.04, update the rules with following command 
```  
sudo vim /etc/udev/rules.d/96-hokuyo.rules
```        
  Replace from
```    
KERNEL=="ttyACM[0-9]*", ACTION=="add", ATTRS{idVendor}=="15d1", MODE="0666", GROUP="dialout", PROGRAM="/opt/ros/noetic/env.sh rosrun urg_node getID %N q", SYMLINK+="hokuyo_%c"
```    
  To
```    
SUBSYSTEMS=="usb", KERNEL=="ttyACM[0-9]*", ACTION=="add", ATTRS{idVendor}=="15d1", ATTRS{idProduct}=="0000", MODE="666", PROGRAM="/opt/ros/kinetic/env.sh rosrun urg_node getID %N q", SYMLINK+="hokuyo_%c", GROUP="dialout"
```
  
4.1. Run ARTA ( Simulation )
```
make start_arta_sim
```
4.2. Run ARTA ( Real world )
```
make start_arta
```  
4.2.1. Run ARTA ready RVIZ ( Real world )
```
make arta_rviz
``` 
4.3. Control ARTA with keyboard input
```
make arta_kepboard
```  
4.4. Access current container [received root in default when using Docker container]
```
docker exec -it arta_c bash
```
  Source the workspace after inside the container
```
source /opt/ros/kinetic/setup.bash; source /ros_ws/devel/setup.bash
```
  Exit the Docker container
```
exit
```
4.5. Stop the Docker container
```
make stop
```
5.1. Joystick control

5.1.1. Joystick direct control
```
make arta_control_joy_direct
```
5.1.2. Joystick control with object advoidance [To have effect, require step 5.3 and 6.1]
```
make arta_control_joy
```
5.2. Create and save a map
```
make arta_create_map

make arta_save_map
```
5.3.1. Localisation with AMCL (without a map, solely using Odom)
```
make arta_odom_without_map
```
5.3.2.1. Localisation with AMCL with map
```
make arta_odom_with_map
```
5.3.2.2. Localise with global initial
```
make arta_odom_global_initial
```
6.1. Assistive Navigation
```
make arta_assi_nav
```
6.2. Send goal through RVIZ

Set the tool properties of 2D Nav Goal > Topic to /arta_navigation/move_base_simple/goal

Credit:
```
Author: Chun Iao Tai

This docker file is available with the help from

Rodrigo Chacon Quesada, Cédric Goubard, Sebastian Aegidius  
```  
---------------------------------------------------
### Steps for manual install steps of the ARTA in a new Ubuntu 16.04 environment:
1. Install ROS kinetic according to ROS official website
```
http://wiki.ros.org/kinetic/Installation/Ubuntu
```
2. Install ARTA depends package manually
```
sudo apt-get update && sudo apt-get upgrade && sudo apt-get install -y \
    ros-kinetic-rtabmap-ros \
    ros-kinetic-joy \
    ros-kinetic-joystick-drivers \
    ros-kinetic-global-planner \
    ros-kinetic-map-server \
    ros-kinetic-hector-slam \
    ros-kinetic-slam-gmapping \
    ros-kinetic-amcl \
    ros-kinetic-scan-tools \
    ros-kinetic-urg-node \
    ros-kinetic-phidgets-imu \
    ros-kinetic-imu-complementary-filter \
    ros-kinetic-twist-mux \
    ros-kinetic-teleop-twist-joy \
    ros-kinetic-base-local-planner \
    ros-kinetic-move-base \
    ros-kinetic-urg-c \
    ros-kinetic-laser-proc \
    python-serial \
    ros-kinetic-laser-filters \
    ros-kinetic-teleop-twist-keyboard \
    git\
    --no-install-recommends
```
3. Create a catkin workspace and clone ARTA package
```
mkdir ~/ros_ws
mkdir ~/ros_ws/temp
git -C ~/ros_ws/temp clone https://github.com/Thomas-Tai/arta_docker.git
sudo apt-get install unzip
unzip ~/ros_ws/temp/arta_docker/arta.zip -d ~/ros_ws	
rm -rf ~/ros_ws/temp
cd ~/ros_ws/arta && mv * ../
rm -rf ~/ros_ws/temp
rm -rf ~/ros_ws/arta
```
4. Install rules on local host
```
mkdir /etc/udev/rules.d/; sudo cp ros_ws/SWC_rules/* /etc/udev/rules.d/; sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
```
5. All is done and welcome to use ARTA!
---------------------------------------------------
### Steps for using the ARTA package(required to use this source from this github):
1.1. Launch the ARTA base system
```
roslaunch arta arta.launch
```
1.2. Launch the ARTA base system in simulation
```
roslaunch arta arta.launch simulation:=true
```
2.1. Issuing direct manual control by joystick
```
roslaunch arta_control arta_control.launch direct_control:=true
```
2.2. Issuing direct manual control by keyboart
```
rosrun msc_social_navigation teleop_twist_keyboard.py
```
2.3. Issuing control with object avoidance by joystick (Require Localisation(Step 4.2) and Navigation module(Step5))
```
roslaunch arta_control arta_control.launch
```
3. Creat a map for current environment
```
roslaunch arta_navigation arta_localisation.launch slam_method:=gmapping
```
3.1. Save the map (map), move the map to src\localisation\maps, change map path at src\arta_navigation\launch\arta_localisation.launch
```
rosrun map_server map_saver -f NameOfMap map:=/arta_navigation/map
```
3.2. Save the map (hector map), move the map to src\localisation\maps, change map path at src\arta_navigation\launch\arta_localisation.launch
```
rosrun map_server map_saver -f NameOfMap map:=/arta_navigation/hector_map
```
4.1. Localisation without a map (Only use Odom)
```
roslaunch arta_navigation arta_localisation.launch
```
4.2. Localisation with AMCL
```
roslaunch arta_navigation arta_localisation.launch slam_method:=none loc_method:=amcl
```
4.2.1. localise with global initial
```
rosservice call /arta_navigation/global_localization "{}"
```
5. Assistive Navigation (SLAM with goal in RVIZ or object avoidance)
```
roslaunch arta_navigation arta_navigation.launch obs_avoid:=true autonomous:=false
```
5.1. Send goal from RVIZ
```
Set the tool properties of 2D Nav Goal > Topic to /arta_navigation/move_base_simple/goal
```
---------------------------------------------------
### Steps to due with device path not found (sicklms):
1. Plug the ARTA usb cable to laptop
2. reload rules on local host
```
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
```
3. Replug only the two hokuyo lidar usb cables at the back of the wheelchair
4. Check at cd /dev with ls to see whther all three lidar sensors are listed
5. If not, check the following guide to check the serial number for sicklms
---------------------------------------------------
### Steps to due with serial connection issue when components are not found:

This is the procedure I followed to make the udev rules for the SickLidar work properly with the new laptops:
```
$ ls /dev/
•	No sicklms device shown
$ lsusb

        Bus 001 Device 011: ID 0403:6001 Future Technology Devices International, Ltd FT232 USB-Serial (UART) IC
        Bus 001 Device 009: ID 0403:6001 Future Technology Devices International, Ltd FT232 USB-Serial (UART) IC

$ udevadm info -q all -n /dev/bus/usb/[Bus]/[Device]

•	Example: $ udevadm info -q all -n /dev/bus/usb/001/011
•	Find E: DEVPATH=/devices/blablabla/blablabla/...

$ udevadm info -a -p [DEVPATH]

•	Example: $ udevadm info -a -p /devices/pci0000:00/0000:00:14.0/usb1/1-1/1-1.1/1-1.1.2
•	Find ATTR{serial}="XXXXXXXX"
•	Example: ATTR{serial}=="FTAJM13B"

$ sudo gedit /etc/udev/rules.d/97-sicklms.rules

•	See if ATTR{serial}=="XXXXXXXX" matches the one written in this file. If it doesn't change it accordingly.

$ sudo udevadm control --reload-rules

$ sudo udevadm trigger

•	Disconnect USB cable and connect again

$ roslaunch arta arta.launch

If you get the following error:

A Timeout Occurred! 2 tries remaining
A Timeout Occurred! 1 tries remaining
A Timeout Occurred - SickLIDAR::_sendMessageAndGetReply: Attempted max number of tries w/o success!
A Timeout Occurred! 2 tries remaining
A Timeout Occurred! 1 tries remaining
A Timeout Occurred - SickLIDAR::_sendMessageAndGetReply: Attempted max number of tries w/o success!
A Timeout Occurred! 2 tries remaining

Try following the same procedure but with the second [Bus]/[Device] listed after $ lsusb for the USB-Serial converter.

Note: After the components showed with $ ls /dev/ , may need to modify the ROS package to include the components
```

---------------------------------------------------
Original resource from https://github.com/ImperialCollegeLondon/arta

# ARTA

Repository for the Assistive Robotic Transport for Adults (ARTA), a robotic wheelchair that runs on the ROS-Kinetic distro.

If you simply wish to run demos with ARTA or want complete examples of its use, refer to the "Demos" section at the bottom of this README.

## Additional Repositories

To operate this 'smart' wheelchair, the relevant repositories for [localisation](https://github.com/ImperialCollegeLondon/prl_localisation) and [navigation](https://github.com/ImperialCollegeLondon/prl_navigation) must also be included in your catkin workspace. Follow the instructions in those repositories before carrying on with the below.

The URDF description of ARTA (contained in `arta_description`) can also represent a human model, so the [human_models](https://github.com/ImperialCollegeLondon/human_models) package should also be included in your catkin workspace.

For Gazebo simulation of ARTA, it will be necessary to include the [gazebo_utilities](https://github.com/ImperialCollegeLondon/gazebo_utilities) repository in your catkin workspace.

There are also four input control interfaces available. One of them is the standard joystick controller, whilst the other three rely on your eye movements and require the [wheelchair_controllers](https://github.com/ImperialCollegeLondon/wheelchair_controllers.git) to be cloned into your catkin workspace. You only require this additional repository if you plan to run eye-controlled wheelchair navigation.

## Build Instructions

Clone this repository into your `catkin/src` directory and make sure all submodules (i.e. stack dependencies not available yet for ROS-Kinetic) are up-to-date:
```shell
git submodule update --init --recursive
```

After performing this step, you should be able to build the ARTA package in your catkin workspace (e.g. using `catkin_make` or `catkin build`).

Once you have sourced your "devel" space, make sure to install all dependencies before launching the packages in this repository. You can use a command like this across your entire workspace:
```shell
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

Or alternatively for a specific package:
```
rosdep install package_name --ignore-src --rosdistro $ROS_DISTRO -y
```

This will install any missing ROS dependencies or notify you of missing packages from your "src" space (developed in the lab). If you cannot find some of these packages in rosdep (maybe due to your distro version) then just carry on with the following steps until you reach the "ARTA Launch" section. 

Contact one of the developers if you encounter further issues in the instructions on how to launch ARTA.

### UDEV Rules Setup

For the *first time* configuration on your machine, UDEV rules will need to be setup manually from within the `catkin_ws` directory:
```shell
sudo cp src/arta/arta/udevrules/* /etc/udev/rules.d/
```

Then restart and trigger these updates:
```shell
sudo udevadm control --reload-rules
sudo udevadm trigger
```

These udev rules specify what names to give the USB devices connected to your machine.

### General Guidelines

Once everything is setup properly, you can prepare ARTA for operation by following these steps:

1. Release the safety button on the emergency switch (giant red button)
2. Shift drive sticks from N to D (just below seat) on both sides of the wheelchair
3. Press the switch at the back of wheelchair and a blue light should come on

Whenever you have finished with ARTA, kill all the programs, switch off the back switch and press down on the safety button. Please ensure that no power is running through the wheelchair (the back switch and joystick buttons should remain inactivated).

If battery is low, as indicated by the joystick's display (orange-to-red colour), then connect the portable lead-acid battery charger into the port just below the joystick. This process will require you to leave the safety button on, so as to allow the wheelchair to charge. **Important:** The joystick's indicator will display a charging behaviour when the power supply is simply plugged in, this does _NOT_ mean that ARTA is necessarily charging. Look at the actual battery charger and a foggy yellow light should be on to validate that the powered wheelchair is charging.

## ARTA Launch

The following sections cover the different ways of launching ARTA. Here is a breakdown of the packages contained in this repository:
- `arta` contains the main controller nodes and launcher for the robotic wheelchair (both in reality and simulation)
- `arta_control` generates velocity control commands given some input interface e.g. a joystick, eye gaze etc.
- `arta_description` provides the complete 3D robot model description of ARTA
- `arta_meta` is the metapackage for this repository
- `arta_msgs` are any user-defined messages that are ARTA-specific
- `arta_navigation` is a package comprised of localisation and navigation launchers for ARTA

### Main Controller & Sensors

Prior to any robot localisation or navigation, the main controller and sensor nodes can be launched using the following command:
```shell
roslaunch arta arta.launch
```

This will activate all the components that are required to operate ARTA.

#### Connection Pitfalls

If you obtain an error regarding the fact you are unable to open a port for Hokuyo, check hokuyo ports in `arta.launch`. As of 12/09/18, we should have:
- front_left: H1009076
- front_right: H0905371

If you get an error regarding lack of connection to the Arduino, check the cable that enters the control box near the joystick. This connection can be fickle.

#### RViz

RViz can also be configured to visualise the topic data spat out by ARTA if you append the `rviz:=true` argument.

There are numerous `.rviz` files available in the `rviz` directory depending on your use-case of ARTA.

### Gazebo Simulation

To instead launch the ARTA platform in Gazebo simulation, you can run:
```shell
roslaunch arta arta.launch simulation:=true
```

This will make use of the [gazebo_utilities](https://github.com/ImperialCollegeLondon/gazebo_utilities) repository and load the `gazebo_utilities/worlds/lab_hall.world` file to spawn our ARTA model (defined by the `arta_description` package) in the Gazebo GUI client. 

The Gazebo world is based on a heightmap that has been constructed from the 2D occupancy grid `lab_hall.png` in the `gazebo_utilities/materials/textures` directory. To load this heightmap, you **must** first copy your `.png` file into Gazebo's `media/materials/textures` directory. You might have to also *invert* the image to obtain the desired heightmap effects (refer to [gazebo_utilities](https://github.com/ImperialCollegeLondon/gazebo_utilities) for more information).

### Issuing Control

In order to issue velocity commands to ARTA, a control loop will need to be established via the `arta_control` package:
```
roslaunch arta_control arta_control.launch
```

By default, this package assumes assistive navigation is yet to be provided, however if you want direct manual control of the wheelchair then append the `direct_control:=true` argument.

The `arta_control/config` directory also contains parameter `.yaml` files to specify how to interpret different joystick controllers and convert them into "Twist" velocity messages.

#### Exotic Control Interfaces

This sub-section covers how to launch the exotic controllers found in the [wheelchair_controllers](https://github.com/ImperialCollegeLondon/wheelchair_controllers.git) repository (refer there for initial setup instructions).

Effectively each of the gaze-based modes of control can be triggered via the `gaze_to_vel` and `gaze_mode` arguments, e.g. for the screen-based method (`screen_control` package):
```shell
roslaunch arta_control arta_control.launch gaze_to_vel:=true gaze_mode:=screen
```

The Pupil Capture application will also need to be running, in which a surface must be defined and named as "screen_control".

Similarly, the free-viewing gaze controller (`gaze_control` package) can be launched as follows:
```shell
roslaunch arta_control arta_control.launch gaze_to_vel:=true gaze_mode:=pupil
```

There is no need to define a surface for this gaze input method, the Capture application and [Pupil_ROS](https://github.com/ImperialCollegeLondon/Pupil_ROS) nodes must still be running though.

Finally, the webcam-based controller (`webcam_control` package) is default launched as:
```shell
roslaunch arta_control arta_control.launch gaze_to_vel:=true
```
The webcam-based controller will also need the `rt_gene` argument in `arta.launch` set to "true.

**Note:** The joystick will still be functional across all interfaces unless `joy_to_vel` is set to "false".

### Robot Navigation

To make use of the `arta_navigation` package, the [localisation](https://github.com/ImperialCollegeLondon/prl_localisation) and [navigation](https://github.com/ImperialCollegeLondon/prl_navigation) repositories are required.

#### Localisation

For SLAM, the `slam_method` argument of the `arta_localisation.launch` can be set to either "hector", "gmapping", "cartographer" or "rtabmap" depending on your use-case.
If a map is already available for use (e.g. from the `maps` directory of the [localisation](https://github.com/ImperialCollegeLondon/prl_localisation) repository), then pure localisation can take place by assigning the `loc_method` argument to either "amcl" or "rtabmap", and setting `slam_method:=none`. 

For instance, you can launch ARTA with RGB-D SLAM capabilities as follows:
```shell
roslaunch arta_navigation arta_localisation.launch slam_method:=rtabmap
```

Or for pure RGB-D localisation:
```shell
roslaunch arta_navigation arta_localisation.launch slam_method:=none loc_method:=rtabmap
```

The `arta_localisation.launch` is primarily responsible for broadcasting both the `odom` (via Hector Mapping) and `map` frames.

#### Assistive Navigation

Once ARTA is localised, it can be operated with assistive navigation as follows:
```shell
roslaunch arta_navigation arta_navigation.launch
```

There are a couple of notable arguments for this launcher:
- `obs_avoid:=true` enables obstacle avoidance to be performed within the `shared_control` package and makes use of `reactive_assistance`
- `autonomous:=true` enables autonomous navigation via the `move_base` package and is configured according to the parameters specified in the `arta_navigation/config/move_base` directory

If neither of these arguments are set to "true", the wheelchair will just operate with manual control.

See the [navigation](https://github.com/ImperialCollegeLondon/prl_navigation) repository for more information on each of the aforementioned packages and visualise their implemention in RViz using the `arta_navigation.rviz` configuration.

## Demos

This section covers how to launch complete demos with ARTA.

**3D-LiDAR with RT-GENE** 

1) Start your `roscore` and then run in a separate terminal:
```shell
roslaunch arta arta.launch rt_gene:=true use_3d_lidar:=true rviz:=true rviz_file:=arta_demo.rviz
```

2) Launch the ARTA control system in a separate terminal to allow for both joystick and gaze control (using [RT-GENE](https://github.com/Tobias-Fischer/rt_gene)) of the wheelchair:
```shell
roslaunch arta_control arta_control.launch gaze_to_vel:=true
```

3) Launch in separate terminals the localisation and navigation packages:
```shell
roslaunch arta_navigation arta_localisation.launch
roslaunch arta_navigation arta_navigation.launch
```

The `arta_demo.rviz` file is an RViz configuration file with the 3D LiDAR readings and gaze estimation outputs prepared for visualisation during the demo.

Also note that the joystick takes precedence over eye gaze input commands during wheelchair navigation.
