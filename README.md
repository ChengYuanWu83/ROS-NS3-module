## Install pre-requisites

### Clone this repo

You may fork this and substitute the following cloned url
```shell
$ git clone git@github.com:ChengYuanWu83/ROS-NS3-moudule.git
```
### Install ROS

You can You can also follow the instruction on ROS [official page] on ROS [offical page](https://wiki.ros.org/Installation/Ubuntu).
We tested our program on Ubuntu 20.04 with ROS Noetic. If you are using a different OS, please select the corresponding ROS version.
```shell
# Set up your keys
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt install curl # if you haven't already installed curl
$ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
$ sudo apt update
# ROS installation
sudo apt install ros-noetic-desktop-full
```

