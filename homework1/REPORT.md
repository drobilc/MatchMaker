# Observation model
When evaluating our model performance, we focused on two things - face detection speed and number of frames where our face was detected. We have tested three different face detectors, two from dlib library and one from opencv.

## Speed
All the tests were done on realtime video that was streaming to the `/camera/raw_image` (similar to the turtlebot configuration that we will be using for our final task). We could process each frame individually, but that would defeat the purpose of our face detector. We wanted to make sure, that the it can be run realtime on our robot and produce good results. 

In the opencv implementation, we are using haar cascade, that was and is widely used in cameras as it is very fast and can be run in realtime.

We have also tested dlib's HOG and CNN detector. The HOG detector is extremly fast, the fastest of the three. We have, on the other hand, observed, that the dlib's convolutional neural network face detection is slow and cannot be run in realtime.

In the table below, we compare the processing speed of different detectors. We are interested in how many frames, the algorithm can process and the number of frames that the face was detected in. If the resolution of the image is too low, the faces will most likely not be detected at all, so we also count how many faces were detected in processed frames.

| Detector | Frame size | Processed frames | Face detections |
| -------- | ---------- | ---------------- | --------------- |
| dlib HOG | 1920 x 1080 | 22 / 130 | 9 |
| dlib HOG | 960 x 540 | 69 / 130 | 18 |
| dlib HOG | 480 x 270 | 130 / 130 | 18 |
| dlib CNN | 1920 x 1080 | 1 / 130 | 0 |
| dlib CNN | 960 x 540 | 1 / 130 | 0 |
| dlib CNN | 480 x 270 | 4 / 130 | 0 |
| opencv haar cascade | 1920 x 1080 | 8 / 130 | 8 |
| opencv haar cascade | 960 x 540 | 26 / 130 | 18 |
| opencv haar cascade | 480 x 270 | 90 / 130 | 29 |

As we can see, the dlib HOG detector is the fastest - it can process frames in real time. The dlib CNN detector is the slowest - it can only process one frame before the video ends. This shows that the CNN detector can not be used to detect faces in our finished product.

THe opencv is not quite as fast as the dlib HOG detector, but it can process data almost in realtime and it actually detects the face very well - even better than HOG.

The Kinect video resolution is 640 x 480, which means that we will be able to process it using either HOG or haar cascade detector.

## Face detection
