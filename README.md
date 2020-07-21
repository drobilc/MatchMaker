<p align="center">
  <img height="150" src="https://github.com/drobilc/MatchMaker/blob/master/documentation/logo.png" alt="MatchMaker logo">
</p>

# Match Maker

short catchy description here

## About

Erazem, a robot with matchking skills even better than Tinder, was hired by Gargamel, who wanted to find love and get married as quickly as possible. Erazem first had to find Gargamel to ask him about his preferences. He already knew that Gargamel was looking for a woman, but since usually his clients also cared about the looks, he also wanted to find out about this kind of preferences.
	
When Gargamel surprised him with his simple answer, that he only cared about the hair colour and hairstyle, he couldn't help but think to himself, what a strange man Gargamel was. He shook off this thought since he had had much stranger clients before and started searching right away.

He was scanning the place for the women. For each face he saw, he quickly filled his mental checklist with Gargamel preferences. "Such easy criteria", he thought to himself, "but only a handful of matches..." When he finally came across a woman that matched Gargamel's preferences, he fixed his tie, took a deep breath, and approached her. She seemed interested in meeting Gargamel but was very secretive. When Erazem asked her to tell him something about herself, she only told him her favourite colour. Erazem firmly remembered this little detail and after some chit-chat said goodbye to her. He was happy to have finally found a potential suitor but at the same time a little worried about the lack of information he had about her.

Erazem thought it is time to report to Gargamel about his findings. They met at Gargamel's and Erazem told him all he knew about that woman. It was up to Gargamel to decide whether she seemed promising or not. If the Gargamel was to decline this suitor, Erazem would have gone out again and found him another one. When Gargamel liked the suitor Erazem suggested, he told him a bit nervously that he believed that tossing a coin into the wishing well and saying a wish makes that wish come true. Excellent matchmaker as Erazem was, he immediately understood. He agreed to find a wishing well in the woman's favourite colour and toss a coin in it while also saying the wish for Gargamel and the woman to marry and live happily ever after.

Erazem searched through his memory whether he has already seen a well of this colour. If he could remember, he would have definitely known where to go. After some time, he found the right wishing well, tossed the coin and said the wish. Now he went about to find a perfect ring. Of course, it had to be in the woman's favourite colour since Gargamel didn't have much opinion on which colours he liked. He picked it up and brought it to the woman.

This was the moment of truth. Will she agree to marry Gargamel, having received such thoughtful gift? If she agrees, Erazem's work here is done. But what if she isn't impressed? Well, then Erazem will find another potential suitor and charge Gargamel some extra money, for the inconvenience.

He knew that his job was very impactful. He promised himself not to let Gargamel down. And the reward at the end will be beautiful. Two lost souls finding each other and living their happy ever after.

## Features

In short, robot autonomously moves around, detects and approaches faces, rings and cylinders, recognizes faces, synthetizes and recognizes speech.

All detections are marked: pink spheres mark detected faces, spheres in other colours mark detected rings, and cylinders mark cylinders. Yellow cubes mark the goals for robot to move to.

Details of implementation can be found [here](https://github.com/drobilc/MatchMaker/tree/master/documentation/final_report/final_report.pdf).

Here you can see how all of this looks in action:

add photos here

## Bugs
  * Erazem can sometimes get stuck and loops infinitely while tossing a coin into a wishing well. This happens because the robot isn't able to reach the desired approaching point This could be solved by implementing a time-out.
  * If the robot can't get to the approacing point, it can't pick up a ring and the simulation must be restarted.
  * The robot also sometimes randomly stops. This is due to the fact that some of the exploration points are not reachable. Here, the time-out was implemented, so the robot can continue with its path.

- kaj dela in kaj ne zares (od naše kode)
- v okviru česa je blo narjeno
- ker node se s čim ukvarja
- a nardimo tud readmeje za vsak package posebi?
- zbrisat mormo nerelevantne reameje in nerelevantne pakete
- kak screenshot našga robotka v akciji
- kako se menja svet

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

You can run everything by moving into your ROS workspace folder and using a command `roslaunch love_maker_2000 all.launch`.
