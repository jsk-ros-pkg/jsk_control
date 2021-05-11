#!/usr/bin/env python

# 1. estimation_config.launch
#    i) include leg or not, bool
#   ii) use chain in 2nd calibration, bool
#  iii) calibrate joint or not, bool

# 2. free_cb_locations.yaml
#    i) include leg or not, bool

# 3. free_cameras.yaml
#    i) include leg
#   ii) include arm

# we have 6 boolean parameters...

# 4. free_arms.yaml
#   * which joint to calibrate

import os
import sys
import shutil
import stat
import re
from multiprocessing import Pool
import multiprocessing
import subprocess
import time
JOINT_NAMES = ["LARM_JOINT0", "LARM_JOINT1", "LARM_JOINT2", "LARM_JOINT3", "LARM_JOINT4", "LARM_JOINT5", "LARM_JOINT6", "LARM_JOINT7",
               "RARM_JOINT0", "RARM_JOINT1", "RARM_JOINT2", "RARM_JOINT3", "RARM_JOINT4", "RARM_JOINT5", "RARM_JOINT6", "RARM_JOINT7",
               "HEAD_JOINT0", "HEAD_JOINT1",
               "CHEST_JOINT0", "CHEST_JOINT1"]

counter = 1

def replaceStringWithDict(format, variables):
    for (key, val) in variables.items():
        format = format.replace("${" + key + "}", str(val))
    return format

def generateEstimationConfig(root_dir, 
                             estimation_config_include_leg,
                             estimation_config_use_chain,
                             estimation_config_calibrate_joint):
    file_string = """
    <launch>

  <group ns="calibration_config" clear_params="true">
    <rosparam file="$(find jsk_calibration)/staro_calibration/estimate_params/test_all/${root_dir}/config/system.yaml" command="load" />

    <group ns="cal_steps">

      <group ns="staro - 00 - Estimating Checkerboard Locations">
        <param name="free_params" textfile="$(find jsk_calibration)/staro_calibration/estimate_params/test_all/${root_dir}/config/free_cb_locations.yaml" />
        <param name="use_cov" type="bool" value="False" />
        <rosparam if="${estimation_config_include_leg}">
          sensors:
          - LARM_chain
          - RARM_chain
          - LLEG_chain
          - RLEG_chain
          - head_camera
        </rosparam>
        <rosparam unless="${estimation_config_include_leg}">
          sensors:
          - LARM_chain
          - RARM_chain
          - head_camera
        </rosparam>
        <param name="output_filename" type="string" value="config_0" />
      </group>

      <group ns="staro - 01 - Adding Camera Locations">
        <param name="free_params" textfile="$(find jsk_calibration)/staro_calibration/estimate_params/test_all/${root_dir}/config/free_cameras.yaml" />
        <param name="use_cov" type="bool" value="True" />
        <group if="${estimation_config_use_chain}">
          <rosparam if="${estimation_config_include_leg}">
            sensors:
            - LARM_chain
            - RARM_chain
            - LLEG_chain
            - RLEG_chain
            - head_camera
          </rosparam>
          <rosparam unless="${estimation_config_include_leg}">
            sensors:
            - LARM_chain
            - RARM_chain
            - head_camera
          </rosparam>
        </group>
        <group unless="${estimation_config_use_chain}">
          <rosparam>
            sensors:
            - head_camera
          </rosparam>
        </group>
        <param name="output_filename" type="string" value="config_1" if="${estimation_config_calibrate_joint}"/>
        <param name="output_filename" type="string" value="system_calibrated" unless="${estimation_config_calibrate_joint}"/>
      </group>
      <group ns="staro - 02 - Joint Offset" if="${estimation_config_calibrate_joint}">
        <param name="free_params" textfile="$(find jsk_calibration)/staro_calibration/estimate_params/test_all/${root_dir}/config/free_arms.yaml" />
        <param name="use_cov" type="bool" value="True" />
        <rosparam>
          sensors:
          - LARM_chain
          - RARM_chain
          - LLEG_chain
          - RLEG_chain
          - head_camera
        </rosparam>
        <param name="output_filename" type="string" value="system_calibrated" />
      </group>

    </group>

  </group>

</launch>
"""
    replaced_str = replaceStringWithDict(file_string, {
            "root_dir": root_dir, 
            "estimation_config_include_leg": estimation_config_include_leg,
            "estimation_config_use_chain": estimation_config_use_chain,
            "estimation_config_calibrate_joint": estimation_config_calibrate_joint})
    file_path = os.path.join(root_dir, "estimation_config.launch")
    with open(file_path, "w") as f:
        f.write(replaced_str)
    

def generateFreeCBLocations(root_name, 
                            free_cb_locations_use_leg):
    if free_cb_locations_use_leg:
        file_string = """
  transforms:
    LARM_chain_cb: [1, 1, 1, 1, 1, 1]
    RARM_chain_cb: [1, 1, 1, 1, 1, 1]
    LLEG_chain_cb: [1, 1, 1, 1, 1, 1]
    RLEG_chain_cb: [1, 1, 1, 1, 1, 1]

  chains:
    LARM_chain:
      gearing: [0, 0, 0, 0, 0, 0]
    RARM_chain:
      gearing: [0, 0, 0, 0, 0, 0]
    LLEG_chain:
      gearing: [0, 0, 0, 0, 0, 0]
    RLEG_chain:
      gearing: [0, 0, 0, 0, 0, 0]
    head_chain:
      gearing: [0, 0]
    torso_chain:
      gearing: [0, 0]


  rectified_cams:
    head_camera:
      baseline_shift: 0
      f_shift: 0
      cx_shift: 0
      cy_shift: 0

  tilting_lasers: {}

  checkerboards:
    mmurooka_board:
      spacing_x: 0.0
      spacing_y: 0.0
"""
    else:
        file_string = """
  transforms:
    LARM_chain_cb: [1, 1, 1, 1, 1, 1]
    RARM_chain_cb: [1, 1, 1, 1, 1, 1]

  chains:
    LARM_chain:
      gearing: [0, 0, 0, 0, 0, 0]
    RARM_chain:
      gearing: [0, 0, 0, 0, 0, 0]
    LLEG_chain:
      gearing: [0, 0, 0, 0, 0, 0]
    RLEG_chain:
      gearing: [0, 0, 0, 0, 0, 0]
    head_chain:
      gearing: [0, 0]
    torso_chain:
      gearing: [0, 0]


  rectified_cams:
    head_camera:
      baseline_shift: 0
      f_shift: 0
      cx_shift: 0
      cy_shift: 0

  tilting_lasers: {}

  checkerboards:
    mmurooka_board:
      spacing_x: 0.0
      spacing_y: 0.0
"""
    if not os.path.exists(os.path.join(root_name, "config")):
        os.mkdir(os.path.join(root_name, "config"))
    file_path = os.path.join(root_name, "config", "free_cb_locations.yaml")
    with open(file_path, "w") as f:
        f.write(file_string)

def generateFreeCameras(root_dir, 
                        free_cameras_include_leg,
                        free_cameras_include_arm):
    if free_cameras_include_leg and free_cameras_include_arm:
        file_str = """
  transforms:
    LARM_chain_cb: [1, 1, 1, 1, 1, 1]
    RARM_chain_cb: [1, 1, 1, 1, 1, 1]
    LLEG_chain_cb: [1, 1, 1, 1, 1, 1]
    RLEG_chain_cb: [1, 1, 1, 1, 1, 1]
    CARMINE_joint: [1, 1, 1, 1, 1, 1]

  chains:
    LARM_chain:
      gearing: [0, 0, 0, 0, 0, 0]
    RARM_chain:
      gearing: [0, 0, 0, 0, 0, 0]
    LLEG_chain:
      gearing: [0, 0, 0, 0, 0, 0]
    RLEG_chain:
      gearing: [0, 0, 0, 0, 0, 0]
    head_chain:
      gearing: [0, 0]
    torso_chain:
      gearing: [0, 0]

  rectified_cams:
    head_camera:
      baseline_shift: 0
      f_shift: 0
      cx_shift: 0
      cy_shift: 0

  tilting_lasers: {}

  checkerboards:
   mmurooka_board:
      spacing_x: 0
      spacing_y: 0

"""
    elif free_cameras_include_leg and not free_cameras_include_arm:
        file_str = """
  transforms:
    LLEG_chain_cb: [1, 1, 1, 1, 1, 1]
    RLEG_chain_cb: [1, 1, 1, 1, 1, 1]
    CARMINE_joint: [1, 1, 1, 1, 1, 1]

  chains:
    LARM_chain:
      gearing: [0, 0, 0, 0, 0, 0]
    RARM_chain:
      gearing: [0, 0, 0, 0, 0, 0]
    LLEG_chain:
      gearing: [0, 0, 0, 0, 0, 0]
    RLEG_chain:
      gearing: [0, 0, 0, 0, 0, 0]
    head_chain:
      gearing: [0, 0]
    torso_chain:
      gearing: [0, 0]

  rectified_cams:
    head_camera:
      baseline_shift: 0
      f_shift: 0
      cx_shift: 0
      cy_shift: 0

  tilting_lasers: {}

  checkerboards:
   mmurooka_board:
      spacing_x: 0
      spacing_y: 0

"""
    elif not free_cameras_include_leg and free_cameras_include_arm:
        file_str = """
  transforms:
    LARM_chain_cb: [1, 1, 1, 1, 1, 1]
    RARM_chain_cb: [1, 1, 1, 1, 1, 1]
    CARMINE_joint: [1, 1, 1, 1, 1, 1]

  chains:
    LARM_chain:
      gearing: [0, 0, 0, 0, 0, 0]
    RARM_chain:
      gearing: [0, 0, 0, 0, 0, 0]
    LLEG_chain:
      gearing: [0, 0, 0, 0, 0, 0]
    RLEG_chain:
      gearing: [0, 0, 0, 0, 0, 0]
    head_chain:
      gearing: [0, 0]
    torso_chain:
      gearing: [0, 0]

  rectified_cams:
    head_camera:
      baseline_shift: 0
      f_shift: 0
      cx_shift: 0
      cy_shift: 0

  tilting_lasers: {}

  checkerboards:
   mmurooka_board:
      spacing_x: 0
      spacing_y: 0

"""
    elif not free_cameras_include_leg and not free_cameras_include_arm:
        file_str = """
  transforms:
    CARMINE_joint: [1, 1, 1, 1, 1, 1]

  chains:
    LARM_chain:
      gearing: [0, 0, 0, 0, 0, 0]
    RARM_chain:
      gearing: [0, 0, 0, 0, 0, 0]
    LLEG_chain:
      gearing: [0, 0, 0, 0, 0, 0]
    RLEG_chain:
      gearing: [0, 0, 0, 0, 0, 0]
    head_chain:
      gearing: [0, 0]
    torso_chain:
      gearing: [0, 0]

  rectified_cams:
    head_camera:
      baseline_shift: 0
      f_shift: 0
      cx_shift: 0
      cy_shift: 0

  tilting_lasers: {}

  checkerboards:
   mmurooka_board:
      spacing_x: 0
      spacing_y: 0

"""        
    file_path = os.path.join(root_dir, "config", "free_cameras.yaml")
    with open(file_path, "w") as f:
        f.write(file_str)

def generateFreeArms(root_dir,
                     free_arms_use_arm,
                     free_arms_use_head,
                     free_arms_use_chest,
                     free_arms_use_leg):
    file_str = """
  transforms:
    LARM_chain_cb:   [1, 1, 1, 1, 1, 1]
    RARM_chain_cb:   [1, 1, 1, 1, 1, 1]
    LLEG_chain_cb:   [1, 1, 1, 1, 1, 1]
    RLEG_chain_cb:   [1, 1, 1, 1, 1, 1]
    CARMINE_joint:   [0, 0, 0, 1, 1, 1]
"""
    if free_arms_use_arm:
        file_str = file_str + """
    LARM_JOINT0:     [ 0, 0, 0, 0.0, 1.0, 0.0 ]
    LARM_JOINT1:     [ 0, 0, 0, 1.0, 0.0, 0.0 ]
    LARM_JOINT2:     [ 0, 0, 0, 0.0, 0.0, 1.0 ]
    LARM_JOINT3:     [ 0, 0, 0, 0.0, 1.0, 0.0 ]
    LARM_JOINT4:     [ 0, 0, 0, 0.0, 0.0, 1.0 ]
    LARM_JOINT5:     [ 0, 0, 0, 1.0, 0.0, 0.0 ]
    LARM_JOINT6:     [ 0, 0, 0, 0.0, 1.0, 0.0 ]
    RARM_JOINT0:     [ 0, 0, 0, 0.0, 1.0, 0.0 ]
    RARM_JOINT1:     [ 0, 0, 0, 1.0, 0.0, 0.0 ]
    RARM_JOINT2:     [ 0, 0, 0, 0.0, 0.0, 1.0 ]
    RARM_JOINT3:     [ 0, 0, 0, 0.0, 1.0, 0.0 ]
    RARM_JOINT4:     [ 0, 0, 0, 0.0, 0.0, 1.0 ]
    RARM_JOINT5:     [ 0, 0, 0, 1.0, 0.0, 0.0 ]
    RARM_JOINT6:     [ 0, 0, 0, 0.0, 1.0, 0.0 ]
"""
    if free_arms_use_head:
        file_str = file_str + """
    HEAD_JOINT0:     [ 0, 0, 0, 0.0, 0.0, 1.0 ]
    HEAD_JOINT1:     [ 0, 0, 0, 0.0, 1.0, 0.0 ]

"""
    if free_arms_use_chest:
        file_str = file_str + """
    CHEST_JOINT0:     [ 0, 0, 0, 0.0, 0.0, 1.0 ]
    CHEST_JOINT1:     [ 0, 0, 0, 0.0, 1.0, 0.0 ]

"""
    if free_arms_use_leg:
        file_str = file_str + """
    LLEG_JOINT0: [0, 0, 0, 0.0, 0.0, 1.0] 
    LLEG_JOINT1: [0, 0, 0, 1.0, 0.0, 0.0] 
    LLEG_JOINT2: [0, 0, 0, 0.0, 1.0, 0.0] 
    LLEG_JOINT3: [0, 0, 0, 0.0, 1.0, 0.0] 
    LLEG_JOINT4: [0, 0, 0, 0.0, 1.0, 0.0] 
    LLEG_JOINT5: [0, 0, 0, 1.0, 0.0, 0.0] 
    LLEG_JOINT6: [0, 0, 0, 0.0, 0.0, 0.0]
"""
    file_str = file_str + """
  chains:
    LARM_chain:
      gearing: [0, 0, 0, 0, 0, 0]
    RARM_chain:
      gearing: [0, 0, 0, 0, 0, 0]
    LLEG_chain:
      gearing: [0, 0, 0, 0, 0, 0]
    RLEG_chain:
      gearing: [0, 0, 0, 0, 0, 0]
    head_chain:
      gearing: [0, 0]
    torso_chain:
      gearing: [0, 0]

  rectified_cams:
    head_camera:
      baseline_shift: 0
      f_shift: 0
      cx_shift: 0
      cy_shift: 0

  tilting_lasers: {}

  checkerboards:
    mmurooka_board:
      spacing_x: 0
      spacing_y: 0
"""
    file_path = os.path.join(root_dir, "config", "free_arms.yaml")
    with open(file_path, "w") as f:
        f.write(file_str)
def generateSystemYaml(root_dir):
    file_str = """
base_link: BODY

sensors:

  chains:
    LARM_chain:
      root: CHEST_LINK1
      tip: LARM_cb_jig
      cov:
       joint_angles: [0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01]
      gearing: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    RARM_chain:
      root: CHEST_LINK1
      tip: RARM_cb_jig
      cov:
       joint_angles: [0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01]
      gearing: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    head_chain:
      root: CHEST_LINK1
      tip: HEAD_LINK1
      cov:
       joint_angles: [0.01, 0.01]
      gearing: [1.0, 1.0]
    torso_chain:
      root: BODY
      tip: CHEST_LINK1
      cov:
       joint_angles: [0.01, 0.01]
      gearing: [1.0, 1.0]
    LLEG_chain:
      root: BODY
      tip: lleg_end_coords # LLEG_LINK5?
      cov:
       joint_angles: [0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01]
      gearing: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    RLEG_chain:
      root: BODY
      tip: rleg_end_coords # RLEG_LINK5?
      cov:
       joint_angles: [0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01]
      gearing: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
  rectified_cams:
    head_camera:
      chain_id: head_chain  #TODO: get rid of this
      frame_id: camera_rgb_optical_frame
#      frame_id: HEAD_LINK1
      baseline_shift: 0.0
#      baseline_rgbd: 0.075 ## comment out if we run all_pipelines.launch with use_kinect
      f_shift: 0.0
      cx_shift: 0.0
      cy_shift: 0.0
      cov: {u: 0.25, v: 0.25, x: 0.25}

  tilting_lasers: {}

checkerboards:
  mmurooka_board:
    corners_x: 5
    corners_y: 6
    spacing_x: 0.03
    spacing_y: 0.03

transforms:
  LARM_chain_cb:   [-0.1, 0, 0, 0, 0, 0]
  RARM_chain_cb:   [0, 0, 0, 0, 0, 0]
#default_floating_initial_pose: [0.3, 0, 0.2, -1.2092, 1.2092, -1.2092]
#default_floating_initial_pose: [0, 0, 0, 0, 0, 0]
initial_poses: "/tmp/staro_calibration/config_0_poses.yaml"
# xyz: [-0.002956968077825933, 0.16898467170866147, -0.20050853695600393]
# rpy: (-0.04425102767135782, 0.01876497698337332, -0.019873022766080706)
"""
    with open(os.path.join(root_dir, "config", "system.yaml"), "w") as f:
        f.write(file_str)

def generateSh(root_dir):
    global counter
    port = counter + 11311
    counter = counter + 1
    file_str = """
#! /bin/bash
set +x
rosparam delete /calibration_config
cd ${root_dir}
if [ -f robot_calibrated.xml ]; then
  echo "robot_calibrated.xml already exists. Either back up this file or remove it before continuing"
  exit 1
fi

echo "Checking if we can write to robot_calibrated.xml..."
touch robot_calibrated.xml
if [ "$?" -ne "0" ]; then
  echo "Not able to write to robot_calibrated.xml"
  echo "Make sure you run this script from a directory that for which you have write permissions."
  exit 1
fi
rm robot_calibrated.xml
echo "Success"

# staro_calibration
roslaunch estimation_config.launch
rosrun calibration_estimation multi_step_cov_estimator.py staro_calibration/cal_measurements.bag /tmp/staro_calibration __name:=cal_cov_estimator

est_return_val=$?

if [ "$est_return_val" -ne "0" ]; then
  echo "Estimator exited prematurely with error code [$est_return_val]"
  exit 1
fi

# Make all the temporary files writable
chmod ag+w staro_calibration/*
"""
    replaced_str = replaceStringWithDict(file_str, {"root_dir": root_dir, "port": port})
    with open(os.path.join(root_dir, "calibrate_staro.sh"), "w") as f:
        f.write(replaced_str)
    os.chmod(os.path.join(root_dir, "calibrate_staro.sh"), 0o0755)
        
def copyBagFiles(root_dir):
    os.mkdir(os.path.join(root_dir, "staro_calibration"))
    to_path = os.path.join(root_dir, "staro_calibration", "cal_measurements.bag")
    shutil.copyfile("cal_measurements.bag", to_path)

def genFiles(estimation_config_include_leg,
             estimation_config_use_chain,
             estimation_config_calibrate_joint,
             free_cb_locations_use_leg,
             free_cameras_include_leg,
             free_cameras_include_arm,
             free_arms_use_arm,
             free_arms_use_head,
             free_arms_use_chest,
             free_arms_use_leg):
    directory_name = "test_%s_%s_%s_%s_%s_%s_%s_%s_%s_%s" % (estimation_config_include_leg,
                                                             estimation_config_use_chain,
                                                             estimation_config_calibrate_joint,
                                                             free_cb_locations_use_leg,
                                                             free_cameras_include_leg,
                                                             free_cameras_include_arm,
                                                             free_arms_use_arm,
                                                             free_arms_use_head,
                                                             free_arms_use_chest,
                                                             free_arms_use_leg)
    if "--cleanup" in sys.argv:
        if os.path.exists(directory_name):
            shutil.rmtree(directory_name)
    os.mkdir(directory_name)
    generateEstimationConfig(directory_name, 
                             estimation_config_include_leg,
                             estimation_config_use_chain,
                             estimation_config_calibrate_joint)
    generateFreeCBLocations(directory_name, free_cb_locations_use_leg)
    generateFreeCameras(directory_name, free_cameras_include_leg,
                        free_cameras_include_arm)
    generateFreeArms(directory_name,
                     free_arms_use_arm,
                     free_arms_use_head,
                     free_arms_use_chest,
                     free_arms_use_leg)
    generateSystemYaml(directory_name)
    generateSh(directory_name)
    copyBagFiles(directory_name)


def multipleExecution():
    parallel_num = 32
    # find all calibrate_staro.sh
    sh_files = []
    free_workers = [None] * parallel_num
    for root, dirs, files in os.walk('.'):
        if "calibrate_staro.sh" in files and "latest_calibrated_xml" not in files:
            sh_files.append(os.path.join(root, "calibrate_staro.sh"))
            del dirs[:]
    while len(sh_files) > 0:
        for p, i in zip(free_workers, range(len(free_workers))):
            if p == None or p.poll() != None:
                # we can use the worker
                new_env = os.environ.copy()
                new_env["ROS_MASTER_URI"] = "http://localhost:%d" % (11311 + i + 1)
                process = subprocess.Popen([sh_files.pop()], env=new_env, shell=True)
                free_workers[i] = process
        print("%d tasks are remained"  % (len(sh_files)))
        print(free_workers)
        time.sleep(1)

        

def main():
    if "--execute" in sys.argv:
        multipleExecution()
        return
    for estimation_config_include_leg in (False, True):
        for estimation_config_use_chain in (False, True):
            for estimation_config_calibrate_joint in (False, True):
                for free_cb_locations_use_leg in (False, True):
                    for free_cameras_include_leg in (False, True):
                        for free_cameras_include_arm in (False, True):
                            for free_arms_use_arm in (False, True):
                                for free_arms_use_head in (False, True):
                                    for free_arms_use_chest in (False, True):
                                        for free_arms_use_leg in (False, True):
                                            genFiles(estimation_config_include_leg,
                                                     estimation_config_use_chain,
                                                     estimation_config_calibrate_joint,
                                                     free_cb_locations_use_leg,
                                                     free_cameras_include_leg,
                                                     free_cameras_include_arm,
                                                     free_arms_use_arm,
                                                     free_arms_use_head,
                                                     free_arms_use_chest,
                                                     free_arms_use_leg)

if __name__ == "__main__":
    main()
