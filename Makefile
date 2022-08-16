# What is the usage of this?? present the path of this Makefile?
THIS_FILE := $(lastword $(MAKEFILE_LIST))

.PHONY: help .pull .build .catkin_build start start_arta start_arta_sim arta_control_joy arta_control_joy_direct

help:				## Show this help
	@echo 'Usage: make [target]'
	@echo
	@echo 'Targets:'
	@echo
	@fgrep -h "##" $(MAKEFILE_LIST) | fgrep -v fgrep | sed -e 's/\\$$//' | sed -e 's/##//'

default:			## Clone dependencies, build Docker image and catkin workspace
	@$(MAKE) -s .pull
	@$(MAKE) -s .build
	@$(MAKE) -s .catkin_build

# cannot push some of the files individually to github? PWD meaning?
.pull:				## Clone dependencies
	sudo chmod 757 -R ${PWD}/ros_ws
	@rm -rf ${PWD}/ros_ws
	@rm -rf ${PWD}/ros_ws/temp
	@mkdir ${PWD}/ros_ws
	@mkdir ${PWD}/ros_ws/temp
	#@sudo apt-get install git-lfs
	#@git lfs install
	#@cd ${PWD}/ros_ws/temp && git lfs clone https://Thomas-Tai@bitbucket.org/thomas-tai/arta.git
	@git -C ${PWD}/ros_ws/temp clone https://github.com/Thomas-Tai/arta_docker.git
	@sudo apt-get install unzip
	@unzip ${PWD}/ros_ws/temp/arta_docker/arta.zip -d ${PWD}/ros_ws	
	@rm -rf ${PWD}/ros_ws/temp
	@cd ${PWD}/ros_ws/arta && mv * ../
	@git -C ${PWD}/ros_ws/src clone https://github.com/ros-drivers/driver_common.git
	@rm -rf ${PWD}/ros_ws/temp
	@rm -rf ${PWD}/ros_ws/arta
	# Install rules on local host, as an backup
	@mkdir /etc/udev/rules.d/; sudo cp ros_ws/src/arta/udevrules/* /etc/udev/rules.d/; sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
	@sudo chmod 757 -R /etc/udev/


.build:				## Build Docker image
	docker build --tag=ubuntu-16-ros-kinetic .

.catkin_build:		## Build catkin workspace, fine to use without sudo for cp cmd? ${PWD} is linked to /app
	docker run --detach --rm -v ${PWD}/ros_ws:/ros_ws:rw -v /etc/udev:/etc/udev:rw --name arta_c ubuntu-16-ros-kinetic bash -c "source /opt/ros/kinetic/setup.bash; roscore"
	@sleep 1
	#Udev installed on local host instead
	#@docker exec -it arta_c bash -c "mkdir /etc/udev/rules.d/; cp /ros_ws/src/arta/udevrules/* /etc/udev/rules.d/"
	#; udevadm control --reload-rules; udevadm trigger"
	###---
	#@docker exec -it arta_c bash -c "source /opt/ros/kinetic/setup.bash; cd /ros_ws; catkin config --init --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release; catkin build"
	#####@docker exec -it arta_c bash -c "apt-get update; apt-get install -y ros-kinetic-rtabmap-ros ros-kinetic-joy ros-kinetic-joystick-drivers ros-kinetic-global-planner ros-kinetic-map-server ros-kinetic-hector-slam ros-kinetic-slam-gmapping ros-kinetic-amcl ros-kinetic-scan-tools ros-kinetic-urg-node ros-kinetic-phidgets-imu ros-kinetic-imu-complementary-filter ros-kinetic-twist-mux ros-kinetic-teleop-twist-joy ros-kinetic-base-local-planner ros-kinetic-move-base ros-kinetic-urg-c ros-kinetic-laser-proc python-serial && apt-get autoclean && apt-get autoremove && rm -rf /var/lib/apt/lists/*"
	#ros-kinetic-urg-node ros-kinetic-phidgets-imu ros-kinetic-laser-filters ros-kinetic-imu-complementary-filter 
	#####@docker exec -it arta_c bash -c "pip install pyserial"
	@docker exec -it arta_c bash -c "chmod 757 -R /ros_ws/src; source /opt/ros/kinetic/setup.bash; cd /ros_ws; rosdep install --from-paths src --ignore-src --rosdistro kinetic -y; catkin_make --cmake-args -DBUILD_TYPE=Release"
	#@docker exec -it arta_c bash -c "apt-get install -y wkhtmltopdf"
	@$(MAKE) -s -f $(THIS_FILE) stop

#-v ~/.Xauthority:/root/.Xauthority:rw -v "/tmp/.X11-unix:/tmp/.X11-unix:rw"	--> -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix
# @echo 'Make sure adding /tmp path under Docker Desktop > Resources > File Sharing'
#-v /dev/bus/usb:/dev/bus/usb:rw 
#@echo 'Make sure adding /tmp path under Docker Desktop > Resources > File Sharing'
start:				## Start Docker image and roscore - 	@docker run --detach --rm --net host --privileged -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v ${PWD}/ros_ws:/ros_ws:rw --name arta_c arta bash -c "source /opt/ros/kinetic/setup.bash; source /catkin_ws/kinetic/setup.bash; roscore"
	@echo 'Dont install Docker-desktop or the display will not work!'
	@xhost +si:localuser:root >> /dev/null
	#@xhost +local:docker
	#@xhost local:root
	#docker run --detach --rm --net host --privileged -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v ${PWD}/ros_ws:/ros_ws:rw --name arta_c ubuntu-16-ros-kinetic sleep 1200
	@docker run --detach --rm --net host --privileged -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v ${PWD}/ros_ws:/ros_ws:rw -v /dev:/dev:rw --name arta_c ubuntu-16-ros-kinetic bash -c "source /opt/ros/kinetic/setup.bash; source /catkin_ws/kinetic/setup.bash; roscore"
	#@docker run --detach --rm --net host --privileged -e DISPLAY -v ~/.Xauthority:/root/.Xauthority:rw -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" -v ${PWD}/ros_ws:/ros_ws:rw -v /dev:/dev:rw --name ubuntu16_c ubuntu:16.04 sleep 1200
	#@docker run --detach --rm --network=host --privileged -e DISPLAY -v ~/.Xauthority:/root/.Xauthority:rw -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" -v ${PWD}/ros_ws:/ros_ws:rw --name ubuntu18_c ubuntu:18.04 sleep 1200
	
	@sleep 2

# real world control not successful, need to configure urg_node package
start_arta:			## Start Docker image, roscore and launche arta ROS interface	
start_arta:	start
	-@docker exec -it arta_c bash -c "source /opt/ros/kinetic/setup.bash; source /ros_ws/devel/setup.bash; roslaunch arta arta.launch"
	@$(MAKE) -s -f $(THIS_FILE) stop  # this shouldn't be reached from above, it should stop it but just in case, we'll cleanly exit

start_arta_sim:			## Start Docker image, roscore and launche arta ROS interface	
start_arta_sim:	start
	-@docker exec -it arta_c bash -c "source /opt/ros/kinetic/setup.bash; source /ros_ws/devel/setup.bash; roslaunch arta arta.launch simulation:=true"
	@$(MAKE) -s -f $(THIS_FILE) stop  # this shouldn't be reached from above, it should stop it but just in case, we'll cleanly exit

arta_control_joy:		## enable joy control, not direct control
	@docker exec -it arta_c bash -c "source /opt/ros/kinetic/setup.bash; source /ros_ws/devel/setup.bash; roslaunch arta_control arta_control.launch"

arta_control_joy_direct:		## enable joy control, with direct control
	@docker exec -it arta_c bash -c "source /opt/ros/kinetic/setup.bash; source /ros_ws/devel/setup.bash; roslaunch arta_control arta_control.launch direct_control:=true"

arta_kepboard:			## enable keyboard input to control the arta
	@docker exec -it arta_c bash -c "source /opt/ros/kinetic/setup.bash; source /ros_ws/devel/setup.bash; rosrun msc_social_navigation teleop_twist_keyboard.py"

stop:				## Stop Docker container
	@docker stop arta_c >> /dev/null