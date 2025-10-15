# File: Dockerfile
FROM dustynv/ros:jazzy-ros-base-r36.4.0-cu128-24.04

# Install system dependencies
RUN apt-get update && \
    apt-get install -y python3-pip python3-dev && \
    rm -rf /var/lib/apt/lists/*

# Copy requirements and install
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# Create user (optional, safer than root)
RUN useradd -ms /bin/bash jetson && usermod -aG sudo jetson

USER robot
WORKDIR /home/robot

copy requirements.txt /tmp/requirements.txt
RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install -r /tmp/requirements.txt

COPY ./bash_aliases  .bash_aliases
RUN source /home/robotcont/.bash_aliases

ENV ROS_DOMAIN_ID=10
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
