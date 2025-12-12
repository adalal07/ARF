# Docker Setup

Install Docker Desktop

https://www.docker.com/products/docker-desktop/


Install docker container
`docker pull ros:rolling-perception-noble`


Run bash in docker container
`docker run -it --rm ros:rolling-perception-noble bash`


If you have a mac, you need to run this before running the camera publisher. If you have some other computer you might need to edit this.

```
ffmpeg \
-f avfoundation \
-pixel_format yuyv422 \
-video_size 1280x720 \
-framerate 30 \
-i "0" \
-f mjpeg \
-q:v 5 \
"tcp://0.0.0.0:8081?listen"
```

The following needs to be run inside the docker container.
```
colcon build --packages-select tello
source install/setup.bash
python3 src/tello/camera_publisher.py --camera-index 0 --width 1280 --height 720 --fps 1 &
ros2 run tello aruco_tf_broadcaster --ros-args \
    -p image_topic:=/camera/image_raw \
    -p frame_id:=camera_link \
    -p marker_length:=0.05 &
python3 src/tello/hand_gestures.py &
```
