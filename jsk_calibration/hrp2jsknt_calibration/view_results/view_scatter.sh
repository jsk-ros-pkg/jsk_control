#!/bin/bash
rosrun calibration_estimation error_visualization.py /tmp/hironx_calibration/cal_measurements.bag /tmp/hironx_calibration/ `rospack find hironx_calibration`/view_results/scatter_config.yaml
