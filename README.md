<p align="center">
  <img height="150" src="https://github.com/drobilc/MatchMaker/blob/master/documentation/logo.png" alt="MatchMaker logo">
</p>

# Match Maker

short catchy description here

long descripzion, what robot does, what are the markers, screenshots of Rviz and Gazebo here
- kaj dela in kaj ne zares (od naše kode)
- v okviru česa je blo narjeno
- (avtorji?)
- dodaj nekam link na ros navoila za zbuildat workspace, linke do rosa, rviza in gazebota 
- ker node se s čim ukvarja
- a nardimo tud readmeje za vsak package posebi?
- zbrisat mormo nerelevantne reameje in nerelevantne pakete
- kak screenshot našga robotka v akciji
- kako se menja svet

## Features

## Bugs

## Installation

### System requirements

You have to have [ROS kinetic](http://wiki.ros.org/kinetic/Installation) and python2 installed, as well as [Gazebo](http://gazebosim.org/) and [Rviz](http://wiki.ros.org/rviz/UserGuide). You also need a ROS repository. For ROS kinetic, the compatible version of computer operating system is Ubuntu 16.04, other versions mihght also work but have not been tested with this code.

Since everything runs within a simulation, there is no need for an actual, physical robot. If you really, really want to use a real robot, you'll need a Turtlebot robot with Kinect and a small robotic arm mounted on top (on the right side).

### Get the code

Extract the contents of this repository into the src folder of your ROS workspace.

### Python libraries

Some of the code runs in python2, so you have to haveit installed. Some specific versions of the python libraries are needed. To install them, go to folder containing `requirements.txt` and run command:

```shell
pip install -r requirements.txt
```

### ROS packages

Install the following ROS packages.

```shell
sudo apt-get install ros-kinetic-arbotix
sudo apt-get install ros-kinetic-moveit
sudo apt-get install ros-kinetic-joint-trajectory-controller
sudo apt-get install ros-kinetic-joint-state-controller
sudo apt-get install ros-kinetic-effort-controllers
sudo apt-get install ros-kinetic-position-controllers
sudo apt-get install ros-kinetic-gazebo-ros-control

sudo apt-get install portaudio19-dev
sudo apt install swig
```

### Build the ROS workspace

Finally build your workspace using the following command:

```shell
catkin_make
```

## Running
