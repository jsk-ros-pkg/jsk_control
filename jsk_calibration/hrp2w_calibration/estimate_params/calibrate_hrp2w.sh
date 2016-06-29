#! /bin/bash

port=${1:-11311}
calibdir=${2:-'/tmp/hrp2w_calibration'}

logdir=${calibdir}/$(date +%Y_%m_%d_%H_%M)
mkdir -p ${logdir}
if [ ! -e ${logdir} ]; then
    echo "logdir ${logdir} does not exist"
    exit 1
fi
echo "logdir: ${logdir}"

if [ ! -e ${calibdir}/initial_poses.yaml ]; then
    if [ -e $(rospack find jsk_calibration)/hrp2w_calibration/capture_data/samples/initial_poses.yaml ]; then
        cp $(rospack find jsk_calibration)/hrp2w_calibration/capture_data/samples/initial_poses.yaml ${calibdir}/initial_poses.yaml
    else
        echo "missing initial poses file: $(rospack find jsk_calibration)/hrp2w_calibration/capture_data/samples/initial_poses.yaml"
    fi
fi

if [ -f robot_calibrated.xml ]; then
  echo "./robot_calibrated.xml already exists. Either back up this file or remove it before continuing"
  exit 1
fi

echo "Checking if we can write to ./robot_calibrated.xml..."
touch robot_calibrated.xml
if [ "$?" -ne "0" ]; then
  echo "Not able to write to ./robot_calibrated.xml"
  echo "Make sure you run this script from a directory that for which you have write permissions."
  exit 1
fi
rm robot_calibrated.xml
echo "Success"

# master
export ROS_MASTER_URI=http://localhost:${port}
# hrp2w_calibration
roslaunch jsk_calibration hrp2w_estimation_config.launch
rosrun rviz rviz -d $(rospack find jsk_calibration)/hrp2w_calibration/view_results/pose_guess.rviz &
sleep 5 ## wait rviz
rosrun calibration_estimation multi_step_cov_estimator.py ${calibdir}/cal_measurements.bag ${logdir} __name:=cal_cov_estimator | tee ${logdir}/console_output.log

est_return_val=$?

if [ "$est_return_val" -ne "0" ]; then
  echo "Estimator exited prematurely with error code [$est_return_val]"
  exit 1
fi

# Make all the temporary files writable
chmod ag+w ${calibdir}/*
