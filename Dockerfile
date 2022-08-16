FROM ubuntu:16.04

LABEL maintainer "Sky"

# DEBIAN_FRONTEND noninteractive (force not send error msg image wise) - Not recommend for whole img but can set individu
#ENV DEBIAN_FRONTEND noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO kinetic
# ask for usage?
#ENV MSCL_LIB_PATH /usr/share/c++-mscl

# only activate during build
#ARG DEBIAN_FRONTEND=noninteractive

# using bash cmd instead of sh cmd
SHELL ["/bin/bash", "-c"] 

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    # -f, --force [remove existing destination files]
    ln -s -f /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    # certificate install
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

## certificate for ubuntu 16.04
RUN apt-get update -qq     && apt-get -qq install --no-install-recommends -y apt-utils gnupg wget ca-certificates lsb-release dirmngr

# Setup the sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list

# setup keys
### RUN apt install curl
### RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN for i in 1 2 3; do { apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key 'C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654'; } &&  break || sleep 1; done

# built-in ROS packages - ros-kinetic-desktop-full
RUN sed -i "/^# deb.*multiverse/ s/^# //" /etc/apt/sources.list && \
    apt-get update -qq \
    && apt-get install -y --no-install-recommends ros-kinetic-ros-base \
    && apt-get autoclean \
    && apt-get autoremove \
    # Clear apt-cache to reduce image size
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/kinetic/setup.bash" >> /root/.bashrc

RUN sed -i "/^# deb.*multiverse/ s/^# //" /etc/apt/sources.list && \
    apt-get update -qq \
    && apt-get install -y \
    python-rosdep \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool \
    build-essential \
    # recommend from github
    python-catkin-tools \
    python-pip \
    ssh-client \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init && rosdep update

# install ros packages
RUN apt-get update && apt-get install -y \
    ros-kinetic-ros-core \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get upgrade && apt-get install -y \
    #python3 \
    #python3-pip \
    #python3-rosdep \
    #python3-rosinstall \
    #python3-rosinstall-generator \
    #python3-wstool \
    # Basic utilities
    iputils-ping \
    wget \
    # ROS packages
    ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control ros-kinetic-gazebo9* \
    ros-kinetic-catkin \
    rviz \
    ros-kinetic-controller-manager ros-kinetic-joint-state-controller ros-kinetic-joint-trajectory-controller ros-kinetic-rqt ros-kinetic-rqt-controller-manager ros-kinetic-rqt-joint-trajectory-controller ros-kinetic-ros-control ros-kinetic-rqt-gui \
    ros-kinetic-rqt-plot ros-kinetic-rqt-graph ros-kinetic-rqt-rviz ros-kinetic-rqt-tf-tree \
    ros-kinetic-gazebo-ros ros-kinetic-kdl-conversions ros-kinetic-kdl-parser ros-kinetic-forward-command-controller ros-kinetic-tf-conversions ros-kinetic-xacro ros-kinetic-joint-state-publisher ros-kinetic-robot-state-publisher \
    ros-kinetic-ros-control ros-kinetic-ros-controllers \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-tf \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-rviz \
    libgflags-dev \
    libgoogle-glog-dev \
    build-essential \
    # extra package from ARTA
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
    --no-install-recommends \
    && apt-get autoclean \
    && apt-get autoremove \
    # Clear apt-cache to reduce image size
    && rm -rf /var/lib/apt/lists/*

# Create catkin workspace
ENV ROS_WS=/ros_ws
RUN mkdir -p $ROS_WS/src

# Set the working directory to /arta_ros_ws
WORKDIR $ROS_WS
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && catkin_make -DCMAKE_BUILD_TYPE=Release
    #&& apt-get install xauth
    #&& catkin config --init --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release \
    #&& catkin config --extend /opt/ros/$ROS_DISTRO \
    #&& catkin build

CMD ["bash"]