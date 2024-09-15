#!/bin/bash

set -e

sudo pigpiod

exec $@
