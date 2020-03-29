run_test () {
    # Function parameters: video, frames, detector, downscale, black_and_white
    echo "[" $1 "] Testing configuration: detector = " $3 ", downscale = " $4 ", black_and_white = " $5
    
    # Run the video publisher, save the process id in P1
    timeout 15 roslaunch homework1 video_publisher.launch video_source:=$1 frames:=$2 &
    P1=$!
    
    # Run the face detector, save the process id in P2
    timeout 15 roslaunch homework1 face_detector.launch display_camera_window:=false rotate_image:=true detector:=$3 frames:=$2 downscale:=$4 bw:=$5 video_source:=$1 &
    P2=$!

    # Wait for both processes to finish
    wait $P1 $P2
}

# Find all files inside current folder, this script must be run from videos folder 
videos=`ls *.mp4`

for video in $videos
do
    echo "Testing video: " $video
    
    # Get number of frames of video (requires ffmpeg to be installed)
    frames=$(ffprobe -select_streams v -show_streams $video 2>/dev/null | grep nb_frames | sed -e 's/nb_frames=//')
    echo "Total number of frames: " $frames

    for detector in 2 3
    do
        for downscale in 16 8 4 2 1
        do
            run_test $video $frames $detector $downscale false
            run_test $video $frames $detector $downscale true
        done
    done

done