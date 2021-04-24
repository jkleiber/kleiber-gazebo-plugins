#! /bin/bash

# Need to be in the plugin directory
cd plugins/drivetrain/build

# Add the plugin to the gazebo path
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$(pwd)

# Launch the world
gazebo --verbose ../../../worlds/basic-cardboard-box-world.sdf
