# Love Maker
Love maker is a precious space for development of the basis of the Match Maker, task1.

Love maker will presumably made following the scheme below.
![scheme](https://github.com/drobilc/rins_exercises/blob/master/love_maker/scheme.png "scheme")

#### Face detector
Face detector reads the images from the camera, preprocesses them and runs face detection. It estimates the global position of each face and sends this data to Robustifier.

#### Robustifier
Keeps the global locations of previously detected faces and determines whether the new detection sent by Face detector corresponds to any already discovered faces. If so, it may modify the position of the known face and then send the global coordinates to the MovementController. Otherwise it sends the coordinates that have been received form the Face detector.

#### MovementController
This is the node, responsible for movement. Based on information from the Robustifier it decides wheteher it is time to approach the face. If so, it approaches the face and calls the Greeter. Otherwise it continues it's travel to one of the predetermined locations for space exploration.

#### Greeter
Says hello to the face and tells the MovementController that is has done so.

The components can be run separately using corresponding launch files or simultaneously using the `all.launch` launch file.
```bash
# Running components separately
roslaunch love_maker face_detector.launch
roslaunch love_maker robustifier.launch
roslaunch love_maker movement_controller.launch

# Running components simultaneously
roslaunch love_maker all.launch
```

## Love maker's abilities
Before starting the task:
- Robot should build a map of the given environment

The task:
1. Search the space for faces
    1. Locate the starting position
    2. Define a way to move when searching (hardcode or intelligent)
    3. Detect the faces
        1. ~~Test different detectors in gazebo (fps, detection success rate)~~  **DONE**
        2. Robustify face detector to prevent detecting the same face and considering it as new
        3. Robustyfy face detector to eliminate false positives (sockets, for example)
        4. Put marker on the estimated face location
2. Approach the newly detected face
    1. Move closer to the face and turn towards it
    2. Greet
    3. Mark as already approached
3. Pass detected faces that have already been approached
4. Stop when all the faces have been detected and approached
5. Perform the task as fast as possible

## Face detector
As we found out in `homework1`, the **opencv haar cascade detector** gives us best results in terms of speed and detection rate. So for the first task, we will be using this detector. The code for the first task face detection lives in `love_maker/scripts/face_detector.py`.

```bash
# The display_camera_window argument defaults to false
roslaunch love_maker face_detector.launch display_camera_window:=true
```

Another conclusion drawn from `homework1` is that the images should be **reduced to half** the original kinect resolution and **converted to grayscale** before runing the search for faces. The size of the image that face detector actually proceses is therefore `320 x 240`.

For face detector to be able to convert face positions from image to 3d point, the *map* service must be working.

The face detector publishes the detected positions as *Pose*s to `/face_detections_raw` topic. The robustifier should subscribe to this topic, deduplicate detections and send the real detections to the `/face_detections` topic.

## Robustifier
Subscribes to `/face_detections_raw` and checks if any neighbouring cells already have a detection logged. When a new face is detected, we wait for some more similar detections before deciding that this is not a false positive. After a true positive is confirmed, Robustifier sends the coordinates to which robot should move (not where the face is) to `/face_detections`.

## Run all
To run gazebo, rviz and love maker, run each command in a separate terminal:
1. roscore
2. roslaunch exercise4 rins_world.launch
3. roslaunch exercise4 amcl_simulation.launch
4. roslaunch turtlebot_rviz_launchers view_navigation.launch
5. roslaunch love_maker all.launch