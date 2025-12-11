# Docker Setup

Install Docker Desktop

https://www.docker.com/products/docker-desktop/


Install docker container
`docker pull ros:rolling-perception-noble`


Run bash in docker container
`docker run -it --rm ros:rolling-perception-noble bash`

## Camera publisher

Publish frames from your local webcam into ROS2:

```
python3 src/tello/camera_publisher.py --camera-index 0 --width 1280 --height 720 --fps 30
```

The node publishes `sensor_msgs/msg/Image` messages to `camera/image_raw`. Override the topic/frame id with `--topic` or `--frame-id`, or use standard ROS parameters (`--ros-args -p camera_index:=1`). Stop with `Ctrl+C`.