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

# install raspberry pi packages from source

# install camera_ros
ARG CAMERA_WS=/home/${USERNAME}/camera_ws
RUN if [ "$GUI" = "false" ]; then \
    apt-get update \
    && apt-get install -y pkg-config python3-yaml python3-ply python3-jinja2 openssl libyaml-dev libssl-dev libudev-dev libatomic1 meson \
    && mkdir -p ${CAMERA_WS} \
    && git clone https://github.com/christianrauch/camera_ros.git ${CAMERA_WS}/src/camera_ros \
    && . /opt/ros/humble/setup.sh \
    && rosdep update \
    && rosdep install --from-paths ${CAMERA_WS}/src --ignore-src -y \
    && colcon build --base-paths ${CAMERA_WS} --build-base ${CAMERA_WS}/build --install-base ${CAMERA_WS}/install \
    && rm -rf /var/lib/apt/lists/*; \
fi

    
RUN usermod -aG video ${USERNAME}

COPY entrypoint.sh /entrypoint.sh
COPY bashrc /home/${USERNAME}/.bashrc

ENTRYPOINT [ "/bin/bash", "/entrypoint.sh" ]

CMD ["bash"]
