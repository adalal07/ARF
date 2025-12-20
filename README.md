# ARF: Aerial Robot Friend

This is ARF, your Aerial Robot Friend. We have developed a software packet for the Tello drone model to enable it to (1) Track a dynamic human target and (2) pay attention to gestures as a means to move around.

## Drone + Hardware Setup

In order to run the nodes in this repo, we need to first setup the drone and any related hardware. Before we can even start running the drone, make sure to charge each of the drone batteries (~20 minutes; lasts for 15 minutes of flying).

1) Once we have the batteries, we can place it in the drone and turn it on with the button on the side. The drone LED in the front will flash all colors on startup and then continuously flash ORANGE when broadcasting its WiFi for connection.

2) Find the Tello drone's wifi of the form 'TELLO-XXXX' and connect to it. Now your drone should be connected to your computer and will allow you to send message packets to it. You should see the drone LED turn GREEN.

3) Build and run the code!

## Docker Setup

Install Docker Desktop: https://www.docker.com/products/docker-desktop/

Install docker container
`docker pull ros:rolling-perception-noble`


Run bash in docker container
`docker run -it --rm ros:rolling-perception-noble bash`


## Camera Setup

Take a Logitech camera and connect it via USB to your computer. We need to setup a connection between our local environment and the docker container for live images, so we run the following command. If you are on mac, install ffmpeg using ```brew install ffmpeg```.

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

Once the camera is properly setup, you should see a ring light illuminate around the camera's lens telling you it is streaming. Now, we run the folling camerap publisher inside the docker container.
```
python3 camera_publisher.py --camera-index 0 --width 1280 --height 720 --fps 1 &
```

Now, with the camera publisher running, we can run our ArUco and hand gesture tracking scripts to enable our perception side of the pipeline.

```
colcon build
source install/setup.bash
ros2 run tello aruco_tf_broadcaster --ros-args \
    -p image_topic:=/camera/image_raw \
    -p frame_id:=camera_link \
    -p marker_length:=0.05 &
ros2 run tello hand_gesture_detector &
```

## ArUco Visualizer

In order to visualize what the camera is seeing, we developed our own visualization tool using viser. Viser is commonly used in the space of robotics to visualize point clouds and trajectories, but in this case, we imitated the functionality of rviz2 for ArUco frames. To run the visualizer, follow the commands below.

```
colcon build
source install/setup.bash
ros2 run tello aruco_viser_visualizer
```

Please contact us with any issues or suggestions at adalal542@berkeley.edu. Excited for you to try it out!