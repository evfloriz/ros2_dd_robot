#!/bin/sh

docker image build -t ros2-dd-image-gui --build-arg SIM=true .
