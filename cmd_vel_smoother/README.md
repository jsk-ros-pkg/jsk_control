cmd_vel_smoother
===============

![interpolate](https://cloud.githubusercontent.com/assets/1901008/26520991/c6dfd076-4318-11e7-91ec-8777bb94c1aa.png)

Interpolate velocity command to be published periodically with acceleration / deceleration limit.

### What is this?

Basically this node interpolates velocity not to violate acceleration limit by decelating from last received velocity.

For example PR2 has 10Hz control loop in base_controller and if no `cmd_vel` has come within 0.1 seconds, the controller stops all the moving motors for safety.

If `move_base` node publishes `cmd_vel` lower than 10Hz (let it be 5Hz) but continues publishing, controllers send signal to motors forward for 500ms and stop for 500ms, which causes "rattle" (= moves and suddenly stops repeatedly) and can damages motors and other hardwares.

### Parameters

* `~x_acc_lim` (Double, default: `0.1`)

  limit value of x-axis translational acceleration. This node publishes velocity not to violate this value.

* `~y_acc_lim` (Double, default: `0.1`)

  limit value of y-axis translational acceleration. This node publishes velocity not to violate this value.

* `~yaw_acc_lim` (Double, default: `0.1`)

  limit value of z-axis rotational acceleration. This node publishes velocity not to violate this value.

* `~desired_rate` (Double, default: `10.0`)

  This node interpolates velocity commands to keep the rate

* `~interpolate_max_frame` (Int, default: `5`)

  Max message count to be published from last received velocity command.

### Discussion

- This package is originated from the discussion in https://github.com/ros-planning/navigation/issues/405 .

  I have not looked into detail of implementation but it seems that similar nodes for turtlebot robots exist: https://github.com/yujinrobot/yujin_ocs/tree/devel/yocs_velocity_smoother .

### Author

Yuki Furuta <<furushchev@jsk.imi.i.u-tokyo.ac.jp>>
