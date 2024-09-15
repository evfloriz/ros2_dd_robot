#!/bin/sh

# Docker container run script to enable gui programs
# GPIO access is not needed so --privileged is not used

ROSUSER=ros

docker container run -it --user=$ROSUSER --network=host --ipc=host \
    -v $PWD/dd_robot_ws:/home/$ROSUSER/dd_robot_ws \
    -w /home/$ROSUSER \
    --env=DISPLAY \
    ros2-dd-image-gui
    
