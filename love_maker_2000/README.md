# Love Maker
Love maker is a precious space for development of the basis of the Match Maker, task1.

Love maker will presumably be made following the scheme below.
![scheme](https://github.com/drobilc/rins_exercises/blob/master/love_maker_2000/new_scheme.png "scheme")

#### Face detector
Face detector reads the images from the camera, preprocesses them and runs face detection. It estimates the global position of each face and sends this data to Face mapper.

#### Face mapper


#### Cylinder and ring detector
This 2 in 1 node is responsible for detecting cylinders and rings. It gets information from the point cloud, does the magic and sends location to Robustifier.

#### Robustifier
There are 3 instances of this node, one for each type of objects we are detecting. It keeps the global locations of previously detected objects and determines whether the new detection sent by appropriate detector corresponds to any already discovered object and has been detected enough times to call it a true positive. If so, it publishes a marker and sends the approaching point coordinates to the Movement controller.

#### Map maker
Based on the map it receives, intelligently sets the points for space exploration and sends them to the Movement controller.

#### Movement controller
This is the node, responsible for movement. Based on information from the Robustifier it decides wheteher it is time to approach the object. If so, it approaches the face and calls the Greeter. Otherwise it continues it's travel to one of the predetermined locations for space exploration.

#### Greeter
Says whatever the Movement controller tells it to.

The components can be run separately using corresponding launch files or simultaneously using the `all.launch` launch file. If the `all.launch` file is launched, it also starts Gazebo simulator, amcl simulation and Rviz visualization tool.
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
To run gazebo, rviz and love maker, we can only run one command: `roslaunch love_maker all.launch`.