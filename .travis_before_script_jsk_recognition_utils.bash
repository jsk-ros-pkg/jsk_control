#!/usr/bin/env bash

############################################################
# Setup released packages on shadow-fixed for released test.
############################################################
sudo -H pip install -q rosinstall_generator

rosinstall_generator --tar --rosdistro indigo jsk_recognition_utils >> /tmp/$$.rosinstall

cd ~/ros/ws_$REPOSITORY_NAME/src
wstool merge /tmp/$$.rosinstall
wstool up jsk_recognition/jsk_recognition_utils
