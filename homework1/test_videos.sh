# Timeout in seconds
TIMEOUT=10

# Find all files inside current folder, this script must be run from videos folder 
videos=`ls *.mp4`

for video in $videos
do
    echo "Testing video: " $video

    # First, run our face_detector, then start video stream
    frames=$(ffprobe -select_streams v -show_streams $video 2>/dev/null | grep nb_frames | sed -e 's/nb_frames=//')
    echo "Total number of frames: " $frames
    timeout 10 roslaunch homework1 video_publisher.launch video_source:=$video frames:=$frames &
    P1=$!
    roslaunch homework1 face_detector.launch display_camera_window:=true rotate_image:=true detector:=3&
    P2=$!

    # wait for both processes to finish
    wait $P1 $P2
done