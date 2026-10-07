#! /bin/bash

# Script to create a Xauthority file for the docker container
XAUTH=$HOME/.Xauthority
touch $XAUTH
export XAUTH_LOC=$XAUTH

if [[ "$(uname)" == "Darwin" || $DISPLAY == "" ]]; then
    echo "Running on macOS or no display available. Skipping Xauthority setup."
else
    xhost +local:$USER
fi

