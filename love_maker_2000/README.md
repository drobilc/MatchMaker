# Love Maker
Love maker is a precious space for development of the basis of the Match Maker, task1.

## TODO
* detect colors (Add topic for images to cylinder_segmentation, create color detection service)
* implement new message type (see below)
* implement new structure (see below)
* write new algorithm for setting exploration points
* make ring detection work
* write approaching for faces in Face mapper
* write approaching for cylinders and rings in designated nodes (see below)
* fix scheme
* UPDATE DOCUMENTATION AS YOU DO ANYTHING!

Love maker will presumably be made following the scheme below.
![scheme](https://github.com/drobilc/rins_exercises/blob/master/love_maker_2000/new_scheme.png "scheme")

#### Face detector
Face detector reads the images from the camera (`/camera/rgb/image_raw`), preprocesses them and runs face detection. It sends position of each face in camera frame Face mapper(`detections`).

Subscribed to: `/camera/rgb/image_raw`  
Publishes to: `detections`

#### Face mapper
Gets position of the face from Face detector(`detections`) and calls Localizer service to get the global position of the face and an approaching point. It send them in `marker` format to the Rbustifier(`/face_detections_raw`) and to Visualizer(`markers`).

Subscribed to: `detections`  
Publishes to: `face_detections_raw`, `markers`  
Calls service: Localizer

#### Voxelgrid
Preprocesses point cloud for Cylinder and ring detector.

Subscribes to: `/camera/depth/points`  
Publishes to: `/love_maker_2000/voxelgrid`

#### Cylinder and ring detector
This 2 in 1 node is responsible for detecting cylinders and rings. It gets information from the point cloud through the Voxelgrid (`/love_maker_2000/voxelgrid`). Now it finds the object and calls Color detector service to get it's color. It then sends `marker` message to Visualizer(`markers`) and to appropriate approaching_point_calculator(`approach_cylinder`, `approach_ring`).

Subscribed to: `/love_maker_2000/voxelgrid`, `/camera/rgb/image_raw` (needed for Color detector)  
Publishes to: `markers`, `approach_cylinder`, `approach_ring`  
Calls service: Color detector  
Real name: cylinder_segmentation

#### Color detector (service)
Based on image it receives from Cylinder and ring detector returns the color of the cylinder or ring.

#### Ring approaching point calculator
Computes the approaching point for detected ring from Cylinder and ring detector(`approach_ring`) fills the field for approaching point in the message and send it to appropriate Robustifier(`torus_detections_raw`).

Subscribes to: `approach_ring`  
Publishes to: `torus_detections_raw`

#### Ring approaching point calculator
Computes the approaching point for detected ring from Cylinder and ring detector(`approach_cylinder`) fills the field for approaching point in the message and send it to appropriate Robustifier (`cylinder_detections_raw`).

Subscribes to: `approach_cylinder`  
Publishes to: `cylinder_detections_raw`

#### Robustifier
There are 3 instances of this node, one for each type of objects we are detecting. Face mapper(`face_detections_raw`) and Cylinder and ring detector() send it the locations. It keeps the global locations of previously detected objects and determines whether the new detection sent by appropriate detector corresponds to any already discovered object and has been detected enough times to call it a true positive. If so, it sends the location to Visualizer(`markers`) and approaching point to Movement controller(`face_detections`, `cylinder_detections`, `torus_detections`).

Subscribes to: `face_detections_raw`, `cylinder_detections_raw`, `torus_detections_raw`  
Publishes to: `markers`, `face_detections`, `cylinder_detections`, `torus_detections`

#### Visualizer
Publishes all markers it receives from Face mapper, Cylinder and ring detector and Robustifiers (`markers`).

Subscribes to: `markers`  
Publishes to: `face_markers`, `cylinder_markers`, `torus_markers`, `approach_face_markers`, `approach_cylinder_markers`, `approach_torus_markers`

#### Map maker (service)
Based on the map it receives, intelligently sets the points for space exploration and sends them to the Movement controller.

#### Movement controller
This is the node, responsible for movement. Based on information from the Robustifier(`face_detections`, `cylinder_detections`, `torus_detections`) it decides wheteher it is time to approach the object. If so, it approaches the face and calls the Greeter. Otherwise it continues it's travel to one of the predetermined locations for space exploration.

Subscribes to: `face_detections`, `cylinder_detections`, `torus_detections`  
Publishes to: `goals`  
Calls service: Map maker

#### Greeter (service)
Says whatever the Movement controller tells it to.

#### Marker message type
```JSON
{"approaching_point": "approaching point global stamped pose",
"detection_point": "object locatino global stamped pose",
"object_color": "color of the object or empty if face",
"object_type": "cylinder ring or face"}
```

--------Below this line, the content hasn't been updated yet--------

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