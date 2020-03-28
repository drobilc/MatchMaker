# Perception - Homework 1: Observation model

Link to videos:
- Dropbox: https://www.dropbox.com/sh/7ff0cxeb3ablnm6/AACehzAoGWwcvyqsu9zE2WWEa?dl=0
- Google Drive: https://drive.google.com/drive/folders/12lQJDzARMFnIASCg-pFRf0nFTzMD2tfu?usp=sharing

## Face detector node
There is a minimal face detector inside the `scripts` folder that subscribes to `/camera/rgb/image_raw` topic. It reads an image, uses detector to find faces on image and draw a rectangle around them.

Before you can run the detector, you have to install python library `dlib`. If you don't have `pip` installed, fist run:

```bash
sudo apt install python-pip
sudo apt-get -f install
sudo apt install python-pip
```
Now, you can install `dlib`. Note that this step may take a while.

```bash
pip install dlib
```

You might also need to install cv2 library:
```bash
pip install opencv
```

Now, you can run the detector.

```bash
roslaunch homework1 face_detector.launch

# To display image, you can set the display_camera_window argument to true
roslaunch homework1 face_detector.launch display_camera_window:=true

# You can also tell the face detector that it should rotate image 90 degrees clockwise
roslaunch homework1 face_detector.launch rotate_image:=true
```

If you encounter error on importing the detectors check for missing `__init__.py` file in `scripts/detectors`.

## Video publisher
Before launching the `video_publisher.launch` inside `homework1/launch`, set the default video to play:
- first put your video file inside `homework1/videos` folder
- in `video_publisher.launch` change this fragment by inserting the name of your video:

```xml
<arg name="video_source" value="name_of_your_video" />
```

You can run the publisher like that:

```bash
roslaunch homework1 video_publisher.launch video_source:=<name_of_your_video>

```

Note that you only have to provide the name of your video file, such as `my_video.mp4` since the path to it is already set in the launch file.

