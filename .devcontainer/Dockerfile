FROM ros:rolling-perception-noble

RUN apt-get update && apt-get install -y \
    vim \
    python3-pip \
    python3-venv \
    python3-opencv \
    libopencv-dev \
    ffmpeg \
    ros-rolling-tf-transformations

# USER ros_user

RUN echo 'source /ros_entrypoint.sh' >> ~/.bashrc

# Install regular pip packages
RUN python3 -m pip install --break-system-packages \
    rospkg av numpy pillow pyyaml mediapipe transforms3d viser
    

# Install djitellopy without its opencv-python dependency (we use system opencv)
RUN python3 -m pip install --break-system-packages --no-deps djitellopy
# USER ros_user

EXPOSE 8889
EXPOSE 8890
EXPOSE 11111
EXPOSE 8080

CMD ["bin/bash"]