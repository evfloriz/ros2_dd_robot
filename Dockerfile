#FROM osrf/ros:humble-desktop           for gui, doesn't work on rpi because its only for amd64
FROM ros:humble

# arg to determine which packages to download
ARG GUI=false

# set up packages
RUN apt-get update \
    && apt-get install -y \
    nano \
    wget \
    v4l-utils \
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
    ros-humble-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

# install ROS2 packages for sim and gui
RUN if [ "$GUI" = "true" ]; then \
    apt-get update \
    && apt-get install -y \
    ros-humble-ros-gz \
    ros-humble-ign-ros2-control \
    && rm -rf /var/lib/apt/lists/*; \
fi

# install raspberry pi packages
# copy debian keys from local raspberry pi
# TODO: need to change this to grab from the official source
COPY keys/* /etc/apt/trusted.gpg.d/

# set the apt pkg source list to raspberry pi sources, download, then switch back
RUN if [ "$GUI" = "false" ]; then \
    mv /etc/apt/sources.list /etc/apt/temp \
    && wget http://archive.raspberrypi.org/debian/raspberrypi.gpg.key \
    && mv raspberrypi.gpg.key /etc/apt/keyrings \
    && echo "deb [signed-by=/etc/apt/keyrings/raspberrypi.gpg.key] http://archive.raspberrypi.org/debian/ bookworm main" >> /etc/apt/sources.list \
    && echo "deb http://deb.debian.org/debian bookworm main contrib non-free non-free-firmware" >> /etc/apt/sources.list \
    && echo "deb http://deb.debian.org/debian-security/ bookworm-security main contrib non-free non-free-firmware" >> /etc/apt/sources.list \
    && echo "deb http://deb.debian.org/debian bookworm-updates main contrib non-free non-free-firmware" >> /etc/apt/sources.list \
    && apt-get update \
    && apt-get install -y \
    pigpio \
    rpicam-apps-lite \
    && rm -rf /var/lib/apt/lists/* \
    && mv /etc/apt/sources.list /etc/apt/rpisources.list \
    && mv /etc/apt/temp /etc/apt/sources.list; \
fi

    
RUN usermod -aG video ${USERNAME}

COPY entrypoint.sh /entrypoint.sh
COPY bashrc /home/${USERNAME}/.bashrc

ENTRYPOINT [ "/bin/bash", "/entrypoint.sh" ]

CMD ["bash"]
