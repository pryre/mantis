#!/bin/sh

MANTIS_GZ_DIR="$(rospack find mantis_gazebo)"

export GAZEBO_RESOURCE_PATH="$MANTIS_GZ_DIR/resources:$GAZEBO_RESOURCE_PATH"
export GAZEBO_MODEL_PATH="$MANTIS_GZ_DIR/models:$GAZEBO_MODEL_PATH"
