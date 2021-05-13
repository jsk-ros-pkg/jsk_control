#!/usr/bin/env bash

############################################################
# Setup released packages on shadow-fixed for released test.
############################################################
sudo -H apt-get install -y python-rosinstall-generator

rosinstall_generator --tar --rosdistro indigo jsk_recognition_utils jsk_footstep_msgs >> /tmp/$$.rosinstall

cd ~/ros/ws_$REPOSITORY_NAME/src
wstool merge /tmp/$$.rosinstall
wstool up jsk_recognition/jsk_recognition_utils
wstool up jsk_common_msgs/jsk_footstep_msgs

