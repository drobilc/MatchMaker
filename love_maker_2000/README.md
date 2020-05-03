# Love Maker
Love maker is a precious space for development of the basis of the Match Maker, task1.

## TODO
* detect ring color
* fix color detection on yellow cylinder
* stop the robot when it finds all objects even if it hasn't explored the whole map
* greet cylinders and rings
* make ring detection work
* add approaching point calculation for cylinders
* add approaching point calculation for rings
* add approaching point markers to robustifier
* crop the map
* fine tune exploration points algorithm once all components work
* add launching cylinder and ring detection to all.launch (when the arhitecture is changed)
* make sure all points are in map coordinates before adding them to `ObjectDetection`
* ~~write new algorithm for setting exploration points~~  **DONE**
* ~~detect cylinder color~~  **DONE**
* ~~implement new message type (see below)~~  **DONE**
* ~~change all messages to `ObjectDetection` (Cylinder and ring detector)~~ **DONE**
* ~~add robustifying colors in Robustifier~~  **DONE**
* ~~fix line fitting in approaching for faces in Face mapper~~  **DONE**
* UPDATE DOCUMENTATION AS YOU DO ANYTHING!

Love maker will presumably be made following the scheme below.
![scheme](https://github.com/drobilc/rins_exercises/blob/master/love_maker_2000/new_scheme.png "scheme")

### Face detector
Face detector reads the images from the camera (`/camera/rgb/image_raw`), preprocesses them and runs face detection. It sends position of each face in camera frame Face mapper(`detections`).

Subscribed to: `/camera/rgb/image_raw`  
Publishes to: `detections`

### Face mapper
Gets position of the face from Face detector(`detections`) and calls Localizer service to get the global position of the face and an approaching point. It send them in `ObjectDetection` format to the Rbustifier(`/face_detections_raw`).

Subscribed to: `detections`  
Publishes to: `face_detections_raw`  
Calls service: Localizer

### Voxelgrid
Preprocesses point cloud for Cylinder and ring detector.

Subscribes to: `/camera/depth/points`  
Publishes to: `/love_maker_2000/voxelgrid`

### Cylinder and ring detector
This 2 in 1 node is responsible for detecting cylinders and rings. It gets information from the point cloud through the Voxelgrid (`/love_maker_2000/voxelgrid`). Now it finds the object and calls Color detector service to get it's color. It then sends `ObjectDetection` message to appropriate Robustifier(`cylinder_detections_raw`, `torus_detections_raw`).

Subscribed to: `/love_maker_2000/voxelgrid`, `/camera/rgb/image_raw` (needed for Color detector)  
Publishes to: `cylinder_detections_raw`, `torus_detections_raw`  
Calls service: Color detector

### Color detector (service)
Based on image it receives from Cylinder and ring detector returns the color of the cylinder or ring.

### Robustifier
There are 3 instances of this node, one for each type of objects we are detecting. Face mapper(`face_detections_raw`) and Cylinder and ring detector() send it the locations. It keeps the global locations of previously detected objects and determines whether the new detection sent by appropriate detector corresponds to any already discovered object and has been detected enough times to call it a true positive. If so, it sends `ObjectDetection` message to Movement controller(`face_detections`, `cylinder_detections`, `torus_detections`). It also publishes all markers (`face_markers`, `cylinder_markers`, `torus_markers`). Besides object location it also robustifies the color of the object.

Subscribes to: `face_detections_raw`, `cylinder_detections_raw`, `torus_detections_raw`  
Publishes to: `face_detections`, `cylinder_detections`, `torus_detections`

### Map maker (class)
Based on the map it receives, intelligently sets the points for space exploration and sends them to the Movement controller.

### Movement controller
This is the node, responsible for movement. Based on information from the Robustifier(`face_detections`, `cylinder_detections`, `torus_detections`) it decides wheteher it is time to approach the object. If so, it approaches the face and calls the Greeter. Otherwise it continues it's travel to one of the predetermined locations for space exploration.

Subscribes to: `face_detections`, `cylinder_detections`, `torus_detections`  
Publishes to: `goals`  
Calls service: Map maker

### Greeter (class)
Says whatever the Movement controller tells it to.

### ObjectDetection
```
std_msgs/Header header
geometry_msgs/Pose object_pose
geometry_msgs/Pose approaching_point_pose
std_msgs/ColorRGBA color
string classified_color
string type
```
`type`: `face`, `cylinder`, `ring`  
`classified_color`: `white`, `yellow`, `red`, `blue`, `green`, `black`

## How to run
### All
```
roslaunch love_maker_2000 all.launch
roslaunch love_maker_2000 find_torus_and_cylinder.launch
```

## What does each component do
### Face detector
### Cylinder detector
### Ring detector
### Face mapper
### Ring approaching point calculator

Detection `Pose` received from Ring detector is transformed to a 2D pixel in grid occupied by our map. During transformation, some rounding is done, which has to be considered when transforming back to coordinates. Now we search for the closest wall pixel with regard to detection pixel.  

This wall pixel is then transformed back to coordinates by trying all possible errors caused by rounding to find the closest coordinate of those laying in the pixel (1m ~ 20px). We assume that the closest distance is perpendicular to the wall.  

From detection and wall point computed in previous step, we can compute the orientation vector and shift detection coordinates 9cm in the direction of the orientation vector to get the approacing point coordinates. 

For approaching point orientation, we have to rotate the orientation vector 90 degrees clockwise. When this is done, we send approaching point `Pose` to the Ring detector. 

### Cylinder approaching point calculator

Works similarly as setting approaching points for faces in task1. When a cylinder is detected, Cylinder detector sends cylinder approaching point calculator service a `Pose` with coordinates of the detection. On received detection, robot's position in map coordinates is calculated.  

The difference between the detection's position and the robot's position gives us the orientation vector, which is then transformed to quaternion.  

We calculate approaching point coordinates by shifting detection coordinates 0.5m in the direction of our orientation vector.  

Orientation is set to our orientatin vector. And voila, this is our approaching point `Pose` in map coordinates, which we send back to Cylinder detector.

### Color detector
### Robustifier
### Movement controller


--------Below this line, the content hasn't been updated yet---------------------------------------------------

The components can be run separately using corresponding launch files or simultaneously using the `all.launch` launch file. If the `all.launch` file is launched, it also starts Gazebo simulator, amcl simulation and Rviz visualization tool.
```bash
# Running components separately
roslaunch love_maker face_detector.launch
roslaunch love_maker robustifier.launch
roslaunch love_maker movement_controller.launch

# Running components simultaneously
roslaunch love_maker all.launch
```

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
