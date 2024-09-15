#!/bin/sh

ROSUSER=ros

# GPIO ACCESS TESTS
# -----------------
# mount volume approach
#   -v /dev:/dev \
#   --device-cgroup-rule 'c *:* rmw' \
#   -v /sys:/sys \
# devices and capability approach
#   --device /dev/mem --device /dev/gpiomem --device /dev/gpiochip0 --device /dev/gpiochip1 \
#   --cap-add SYS_RAWIO \
# sledgehammer privileged approach
#   --privileged \

docker container run -it --user=$ROSUSER --network=host --ipc=host \
    -v $PWD/dd_robot_ws:/home/$ROSUSER/dd_robot_ws \
    -w /home/$ROSUSER \
    --privileged \
    ros2-dd-image
