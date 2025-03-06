# ROS-NS3-moudule
This software is created by combining NS-3 network simulator and ROS-based robotic simulator. We tested our program on Ubuntu 20.04 with ROS Noetic and NS-3.40. 
# Install pre-requisites

## Install ROS

You can also follow the instruction on ROS [offical page](https://wiki.ros.org/Installation/Ubuntu)
```shell
# Set up your keys
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt install curl # if you haven't already installed curl
$ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
$ sudo apt update
```
### Pick how much of ROS you would like to install

#### Desktop-Full Install (Recommended) : Everything in Desktop plus 2D/3D simulators and 2D/3D perception packages
```shell
sudo apt install ros-noetic-desktop-full
```
#### ROS-Base: (Bare Bones) ROS packaging, build, and communication libraries
```shell
sudo apt install ros-noetic-ros-base
```

# Install this repo

You may fork this and substitute the following cloned url
```shell
$ git clone git@github.com:ChengYuanWu83/ROS-NS3-moudule.git
```

### Install NS-3
You can also follow the instruction on NS [official page](https://www.nsnam.org/releases/ns-3-40/documentation/).

```shell
# you can change the NS version by editing the 'NS_version' in setup.sh
sh setup.bash
```
Once complete, you can run the unit tests for checking NS3
```shell
./test.py
```
