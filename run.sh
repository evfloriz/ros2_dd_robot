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

# PI CAMERA FIXES
# ---------------
# looks like a udev issue, fixed by this issue
# https://github.com/raspberrypi/picamera2/issues/563#issuecomment-2351092491
# other commands from thread (should be encompassed by --privileged which I need for gpio access):
#   --cap-add=SYS_ADMIN \
#   -v /dev/:/dev/ \
#   --env UDEV=1 \
#   --device /dev/:/dev/ \

docker container run -it --user=$ROSUSER --network=host --ipc=host \
    -v $PWD/dd_robot_ws:/home/$ROSUSER/dd_robot_ws \
    -w /home/$ROSUSER \
    --privileged \
    -v /run/udev/:/run/udev:ro \
    ros2-dd-image
