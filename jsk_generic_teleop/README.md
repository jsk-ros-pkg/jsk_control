# jsk_generic_teleop

`jsk_generic_teleop` provides a Roseus teleoperation loop that keeps input-device handling separate from robot IK.

The main entry point is `jsk-generic-teleop` in `euslisp/jsk-generic-teleop.l`.
It subscribes to one input device, updates target end-effector coordinates, solves IK on the Eus robot model, and sends the resulting angle-vector through the robot interface.


## Supported input devices

- **Touch HID/USB** `:omni`: absolute input from `omni_msgs/OmniState`
- **Space Navigator** `:spacenav`: relative input from `sensor_msgs/Joy`


## Dependencies

The following package is released, please build manually,
[omni_msgs](https://github.com/bharatm11/Geomagic_Touch_ROS_Drivers)


## Usage

Load your robot model and interface first, then call:

```lisp
(require "package://jsk_generic_teleop/euslisp/jsk-generic-teleop.l")
(jsk-generic-teleop
 :robot *robot*
 :arms '(:larm :rarm)
 :device-type :omni
 :init-pose :reset-pose
 :origin-offsets '((:larm . (:pos #f(400 100 50)
                            :rpy-deg #f(0 0 0)))
                   (:rarm . (:pos #f(400 -100 50)
                            :rpy-deg #f(0 0 0))))
 :ratio 0.5)
```

For relative input:

```lisp
(jsk-generic-teleop
 :robot *robot*
 :arms '(:rarm)
 :device-type :spacenav
 :init-pose :reset-pose
 :origin-offsets #f(0 0 0)
 :ratio 1.0)
```

The relative `Joy` convention is axes `[x y z roll pitch yaw]`.
Buttons `0` and `1` select the first and second arm when two arms are passed.

For Touch HID, `origin-offsets` defines where the device zero pose lands in
the robot world frame after `init-pose` is applied. A plain `#f(x y z)` still
works for position-only offsets. Use `(:pos #f(x y z) :rpy #f(roll pitch yaw))` or
`(:pos #f(x y z) :rpy-deg #f(roll pitch yaw))` when the device zero pose also
needs an orientation offset. The default absolute axis mapping follows
`nextage_tutorials/euslisp/jsk-nextage-teleop.l`; pass
`:absolute-rotation-map` when a robot needs a different device-to-robot frame
mapping.
