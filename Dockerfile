FROM ros:humble

SHELL ["/bin/bash", "-c"]

# Install dependencies
RUN apt update && apt install -y \
    python3-pip \
    ros-humble-tf2-ros \
    ros-humble-tf2 \
    ros-humble-rosidl-runtime-py \
    git \
    python3-rosdep

# Install MCAP and other Python dependencies
RUN pip3 install --upgrade pip
RUN pip3 install mcap==1.2.2 h5py numpy scipy pillow mcap-ros2-support>=0.0.8

# Create script directory and copy scripts
RUN mkdir -p /script
COPY ./script /script

# Default working directory for CMD
WORKDIR /
CMD ["/bin/bash", "-c", "echo 'This container is intended to be run via the run_converter.sh script. See README.md for usage. Defaulting to Python script help:' && python3 /script/hdf5_to_mcap_converter.py --help"]