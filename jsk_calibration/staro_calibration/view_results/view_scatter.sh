#!/bin/bash
rosrun calibration_estimation error_visualization.py /tmp/staro_calibration/cal_measurements.bag /tmp/staro_calibration/ `rospack find jsk_calibration`/staro_calibration/view_results/scatter_config.yaml
