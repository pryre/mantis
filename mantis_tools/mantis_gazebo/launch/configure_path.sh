#!/bin/sh

MANTIS_GZ_DIR="$(rospack find mantis_gazebo)"

export GAZEBO_MODEL_PATH="$MANTIS_GZ_DIR/models:$GAZEBO_MODEL_PATH"
export GAZEBO_PLUGIN_PATH="$MANTIS_GZ_DIR/plugins:$GAZEBO_PLUGIN_PATH"
