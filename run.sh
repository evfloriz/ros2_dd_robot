#!/bin/sh

ROSUSER=ros

# mount volume approach
#   -v /dev:/dev \
#   --device-cgroup-rule 'c *:* rmw' \
#   -v /sys:/sys \

# devices and capability approach
#   --device /dev/mem --device /dev/gpiomem --device /dev/gpiochip0 --device /dev/gpiochip1 \
#   --cap-add SYS_RAWIO \

# Sledgehammer privileged approach
#   --privileged \

#-v $HOME/workspaces:/home/$ROSUSER/workspaces \

#-v $HOME/ros2_dd_robot/dd_robot:/home/$ROSUSER/dd_robot_ws/src/dd_robot \
#-v $HOME/ros2_dd_robot/dd_hardware_interface:/home/$ROSUSER/dd_robot_ws/src/dd_hardware_interface \

docker container run -it --user=$ROSUSER --network=host --ipc=host \
    -v $HOME/ros2_dd_robot/dd_robot_ws:/home/$ROSUSER/dd_robot_ws \
    -w /home/$ROSUSER \
    --privileged \
    ros2-dd-image
