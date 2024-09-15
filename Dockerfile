#FROM osrf/ros:humble-desktop           for gui, doesn't work on rpi because its only for amd64
FROM ros:humble

# add raspberry pi os to source list

# set up packages
RUN apt-get update \
    && apt-get install -y \
    nano \
    wget \
    && rm -rf /var/lib/apt/lists/*

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# create non-root user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# set up sudo
RUN apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# install ROS2 packages
RUN apt-get update \
    && apt-get install -y \
    ros-humble-xacro \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

# add pigpio
RUN wget http://archive.raspberrypi.org/debian/raspberrypi.gpg.key \
    && mv raspberrypi.gpg.key /etc/apt/keyrings \
    && echo "deb [signed-by=/etc/apt/keyrings/raspberrypi.gpg.key] http://archive.raspberrypi.org/debian/ bookworm main" >> /etc/apt/sources.list \
    && apt-get update \
    && apt-get install -y pigpio \
    && rm -rf /var/lib/apt/lists/*

COPY entrypoint.sh /entrypoint.sh
COPY bashrc /home/${USERNAME}/.bashrc

ENTRYPOINT [ "/bin/bash", "/entrypoint.sh" ]

CMD ["bash"]
