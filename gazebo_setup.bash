#!/bin/sh

dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${dir}/livox_laser_simulation/models