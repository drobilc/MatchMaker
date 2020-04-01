# Love Maker
Love maker is a testing ground for the first task.

## Face detector
As we found out in `homework1`, the **opencv haar cascade detector** gives us best results in terms of speed and detection rate. So for the first task, we will be using this detector. The code for the first task face detection lives in `love_maker/scripts/face_detector.py`.

```bash
# The display_camera_window argument defaults to false
roslaunch love_maker face_detector.launch display_camera_window:=true
```