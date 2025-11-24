FROM ros:rolling-perception-noble

RUN apt-get update && apt-get install -y \
    vim \
    python3-pip \
    python3-venv

# USER ros_user

RUN echo 'source /ros_entrypoint.sh' >> ~/.bashrc

# RUN python3 -m pip install rospkg djitellopy av opencv

# USER ros_user

CMD ["bin/bash"]