# Perception - Homework 1: Observation model

Link to videos:
- Dropbox: https://www.dropbox.com/sh/7ff0cxeb3ablnm6/AACehzAoGWwcvyqsu9zE2WWEa?dl=0
- Google Drive: https://drive.google.com/drive/folders/12lQJDzARMFnIASCg-pFRf0nFTzMD2tfu?usp=sharing

## Face detector node
There is a minimal face detector inside the `scripts` folder that subscribes to `/camera/rgb/image_raw` topic. It reads an image, uses detector to find faces on image and draw a rectangle around them.

```bash
roslaunch homework1 face_detector.launch

# To display image, you can set the display_camera_window argument to true
roslaunch homework1 face_detector.launch display_camera_window:=true
```

## Video publisher
Before launching the `video_publisher.launch` inside `homework1/launch`, the path to the video has to be set:
- first put your video file inside `gomework1/videos` folder
- in `video_publisher.launch` change this fragment by inserting the name of your video:

```xml
<arg name="video_stream_provider" value="$(find homework1)/videos/name_of_your_video" />
```

