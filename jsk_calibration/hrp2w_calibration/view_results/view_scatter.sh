#!/bin/bash
rosrun calibration_estimation error_visualization.py /tmp/hrp2w_calibration/cal_measurements.bag /tmp/hrp2w_calibration/ `rospack find jsk_calibration`/hrp2w_calibration/view_results/scatter_config.yaml
