# Match Maker

logo here

short catchy description here

long descripzion, what robot does, what are the markers, screenshots of Rviz and Gazebo here

## Installation

### Prerequisites

You have to have ROS kinetic and python2 installed, as well as Gazebo and Rviz. You also need a ROS repository. For ROS kinetic, the compatible version of computer operating system is Ubuntu neki neki, other versions mihght also work but have not been tested with this code.

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