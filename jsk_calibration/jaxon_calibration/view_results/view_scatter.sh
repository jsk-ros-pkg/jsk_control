#!/bin/bash
rosrun calibration_estimation error_visualization.py /tmp/jaxon_calibration/cal_measurements.bag /tmp/jaxon_calibration/ `rospack find jsk_calibration`/jaxon_calibration/view_results/scatter_config.yaml
