# Exercise 4
To use this exercise, extract the zip contents to your ROS workspace.

```bash
# Launch ROS core
roscore

# Launch Gizmo with our 3d models
# I have moved the rins2.world file to our exercise4, so we can still launch the previous world
roslaunch exercise4 rins_world.launch

# Run BaseMove server, be sure to change the map path
roslaunch exercise4 amcl_simulation.launch

# Start the map visualisation
roslaunch turtlebot_rviz_launchers view_navigation.launch

# To run breadcrumbs node execute
rosrun exercise4 breadcrumbs

# To run face_localizer, you first have to install pip and dlib library
sudo apt install python-pip
sudo apt-get -f install
sudo apt install python-pip
pip install dlib
rosrun exercise4 face_localizer

# To run our custom face detector that drives the robot around and
# places markers where faces are, run the face_finder node
rosrun exercise4 face_finder

# To display camera information, we can add argument display_camera_window:=true
rosrun exercise4 face_finder display_camera_window:=true

# To run VICOS face open cv face detector, we first need to install cv library
# then we can run it, notice that we have set the image_topic
# the found faces can be found in /facedetector/faces topic
# if we want to toggle face detection, we can publish to /facedetector/toggle

# If we want to visualize faces, we can edit the show_cv_window parameter inside
# the vicos_ros/detection/opencv_detector/launch/facedetector.launch launch file
roslaunch vicos_ros/detection/opencv_detector/launch/facedetector.launch  image_topic:="/camera/rgb/image_raw"
```

## Face detection
Face detection can be done either using **Viola-Jones cascade detector** or dlib's **HOG detector**. Both detectors are classes inside the `exercise4/scripts/detectors` directory. Each one has a function called `find_faces` that accepts rgb image and returns a list of `Face` objects.

The `HaarDetector` requires a data file (xml file that contains cascade information). This file can be found inside the `exercise4/scripts/detectors/data` directory.

We can change which detector we use by changing the constructor in `face_finder`.

```python
# We can use dlib or haar detector here, just uncomment correct line
# self.face_detector = DlibDetector()
self.face_detector = HaarDetector(self.haar_cascade_data_file_path)
```