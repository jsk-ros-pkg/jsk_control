#!/bin/bash
rosrun calibration_estimation error_visualization.py /tmp/hrp2jsknt_calibration/cal_measurements.bag /tmp/hrp2jsknt_calibration/ `rospack find jsk_calibration`/hrp2jsknt_calibration/view_results/scatter_config.yaml
