# arta_docker

This is the usage for using Docker within Ubuntu 20.04/18.04/16.04

## Steps

0. Install docker engine from official website

  https://docs.docker.com/engine/install/ubuntu/

1. Create a folder (ie. ~/ARTADockerSetup) 

    mkdir ~/ARTADockerSetup

2. Cd to that folder & download Dockerfile and Makefile into the folder

    cd ~/ARTADockerSetup
  
    wget --no-check-certificate --content-disposition https://github.com/Thomas-Tai/arta_docker/raw/main/Dockerfile
  
    wget --no-check-certificate --content-disposition https://github.com/Thomas-Tai/arta_docker/raw/main/Makefile
  
3. Create the Docker image and environment

    make default
  
  3.1. If Host is using 18.04 , update the rules with following command 
  
    sudo vim /etc/udev/rules.d/96-hokuyo.rules
    
  Replace from
    
    KERNEL=="ttyACM[0-9]*", ACTION=="add", ATTRS{idVendor}=="15d1", MODE="0666", GROUP="dialout", PROGRAM="/opt/ros/noetic/env.sh rosrun urg_node getID %N q", SYMLINK+="hokuyo_%c"
    
  To
    
    KERNEL=="ttyACM[0-9]*", ACTION=="add", ATTRS{idVendor}=="15d1", MODE="0666", GROUP="dialout", PROGRAM="/opt/ros/melodic/env.sh rosrun urg_node getID %N q", SYMLINK+="hokuyo_%c"
    
  3.2. If Host is using 16.04, update the rules with following command 
  
    sudo vim /etc/udev/rules.d/96-hokuyo.rules
        
  Replace from
    
    KERNEL=="ttyACM[0-9]*", ACTION=="add", ATTRS{idVendor}=="15d1", MODE="0666", GROUP="dialout", PROGRAM="/opt/ros/noetic/env.sh rosrun urg_node getID %N q", SYMLINK+="hokuyo_%c"
    
  To
    
    SUBSYSTEMS=="usb", KERNEL=="ttyACM[0-9]*", ACTION=="add", ATTRS{idVendor}=="15d1", ATTRS{idProduct}=="0000", MODE="666", PROGRAM="/opt/ros/kinetic/env.sh rosrun urg_node getID %N q", SYMLINK+="hokuyo_%c", GROUP="dialout"

  
4.1. Run ARTA ( Simulation )

    make start_arta_sim

4.2. Run ARTA ( Real world )

    make start_arta
  
4.3. Access current container [received root in default when using Docker container]

    docker exec -it arta_c bash
