# jsk_calibration

This is a utility package for hand/eye calibration using
[ros-perception/calibration](https://github.com/ros-perception/calibration).

This package generates sampling poses automaticaly using
[euslisp](https://github.com/euslisp/jskeus) robot models.

## Each robot documentation
* [HRP2JSKNTS](hrp2jsknts_calibration/README.md)

## Calibrate robot
calibration process has two steps.

1. capture data.

  Move robot and capturing data into a bag file.
  ```sh
  roslaunch capture_data.launch
  ```
2. estimate parameters.

  Estimate calibration parameter from the bag file.
  ```sh
  ./calibrate_${ROBOT}.sh
  ```

  And you will have calibrated urdf file.

  You can check progress of estimation parameters by rviz:
  ```sh
  rosrun rviz rviz -d view_results/pose_guesses.rviz
  ```
  ![pose_guesses](images/pose_guesses.png)

3. If calibration takes a lot of time, please use initial_pose.yaml.
Update your system.yaml by adding `initial_poses: /path/to/initial_pose.yaml`.

## Update sampling poses
* HRP2JSKNT
```lisp
roseus euslis/hrp2-calibration.l
$ (generate-hrp2jsknt-calibration)
```

## Algorithm
1. Sampling several joint angles (for example, head joints).
2. Sampling distance from camera to checker board.
3. Sampling agnles around x, y and z axis of checker board.
4. Solve ik toward target coordinates of checker board.
5. Check self-collision.
6. For all specified sampling joints and angles, compute collision-free poses by 1-5
7. Choose good poses randomly and sparsely
8. Check self-collision for interpolated poses
9. Remove colliding poses
10. Get collision-free and different poses


## Utility scripts
* `joint_states_appender.py`

ros calibration software is not designed to handle multiple joint states.

On the other hand, several robot has multiple joint states, it is that several nodes
publishes `/joint_states`.

In order to resolve this mismatch, `joint_states_appender` subscribes `/joint_states`
and append multiple joint states and publish it into `/joint_states_appended`.
