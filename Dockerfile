FROM debian:buster

SHELL ["/bin/bash", "-c"]

RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y \
       python-dev \
       python-pip \
       python3-dev \
       python3-pip \
       python3-sip-dev \
       git \
       libboost-dev \
       libeigen3-dev \
       liblz4-dev \
       curl \
       sudo

# RUN useradd -m docker && echo "docker:docker" | chpasswd && adduser docker sudo


WORKDIR /work

RUN python3 -m pip install --upgrade pip
RUN pip install --extra-index-url https://rospypi.github.io/simple/ rospy
RUN pip install --extra-index-url https://rospypi.github.io/simple/ tf
RUN pip install --extra-index-url https://rospypi.github.io/simple/ tf2_ros
RUN pip install --extra-index-url https://rospypi.github.io/simple/ visualization_msgs
RUN pip install flask flask_cors

CMD ["bash"]