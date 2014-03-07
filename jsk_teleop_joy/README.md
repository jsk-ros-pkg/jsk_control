# jsk_teleop_joy package

## Introduction
This package provides physical UI for teleoperatoin such as joy sticks,
game controllers and MIDI instruments.

## Supported game controllers
This package supports following game controllers:

1. XBox360 controller
2. PS3 Controller (Bluetooth)
3. PS3 Controller (USB)

In order to use PS3 controller via Bluetooth,
see [this instruction](http://wiki.ros.org/ps3joy/Tutorials/PairingJoystickAndBluetoothDongle).

## Architecture of jsk\_teleop\_joy
jsk\_teleop\_joy (the main script is [`joy_main.py`](scripts/joy_main.py))
is based on plugin architecture.

ROS packages provides the plugins for jsk\_teleop\_joy and each plugin
represents a function such as "specify 6D pose of the end effector".

You can enable some plugins and choose which plugin you want to use by
"Pushing Select Button".

## jsk\_teleop\_joy plugins out of box
* VerboseStatus
* JoyPose6D
* JoyFootstep
* JoyFootstepPlanner
* JoyFootstepPlannerDemo
* JoyMoveIt
* JoyGoPos

## How to implement a plugin
1. write xml file to define plugins.
2. Export the xml file from your ROS package using `manifest.xml` or `package.xml`.
3. implement plugin

### Write xml file to define plugins
jsk\_teleop\_joy reads the plugin definition from a xml file like [`plugin.xml`](plugin.xml).

That xml should be like:
```xml
<library>
  <class name="Foo" type="your_package.foo">
  </class>
</library>
```

The xml file should have `<library>` tag at the top level.
And you can define plugins by `<class>` tag.

* `name` attribute means the name of the plugin. You will choose plugins by this
name in your launch files ([example](launch/joy.launch#L15)).
* `type` attribute means the python class of the plugin. jsk\_teleop\_joy tries
to instantiate plugin class using thie `type` name.

### Export the xml file from your ROS package
You need to export that xml file using `<export>` tag and `<jsk_teleop_joy>` tag
in your `manifest.xml` or `package.xml`.

These two files, [`manifest.xml`](manifest.xml), [`package.xml`](package.xml),
are good examples.

### Implement a plugin
Finally, you can implement a jsk\_teleop\_joy plugin.

[VerboseStatus Plugin](src/jsk_teleop_joy/plugin/verbose.py) is a good example
how to implement a plugin.

#### `__init__` method
All the plugins are required to inherits
`jsk_teleop_joy.joy_plugin.JSKJoyPlugin` and call `JSKJoyPlugin.__init__` in
its `__init__` constructor.

```python
class VerboseStatus(jsk_teleop_joy.joy_plugin.JSKJoyPlugin):
  def __init__(self):
    jsk_teleop_joy.joy_plugin.JSKJoyPlugin.__init__(self, 'VerbosePlugin')
```

#### `joyCB` method
Each time jsk\_teleop\_joy receives `/joy` message, it calls `joyCB` method
of the active plugin.

```python
  def joyCB(self, status, history):
    rospy.loginfo('analog left (%f, %f)' % (status.left_analog_x, status.left_analog_y))
```

The 2nd argument of `joyCB` is an instance of [`JoyStatus`](src/joy_status.py).
`JoyStatus` is one of `XboxStatus`, `PS3Status` and `PS3WiredStatus`.
This means the latest message from `/joy`.
These 3 classes provide the same interface and you don't need to care about
which controller the user uses.

On the other hand, the 3rd argument of `joyCB` (`history`), is a sequence of
`JoyStatus`. it's an instance of
[`StatusHistory`](src/jsk_teleop_joy/status_history.py). It means a hisotry
of recent `JoyStatus`.

### SELECT button
jsk\_teleop\_joy system occupies SELECT button so you cannot use SELECT button
in your plugin. SELECT button is used for switching plugins.

## MIDI controllers
### [`interactive_midi_config.py`](scripts/interactive_midi_config.py)
You can configure MIDI devices by this script interactively.
[movie](http://www.youtube.com/watch?v=1JOKra7gZVs)

0. Connect you MIDI device.
1. First, the script asks the name of device, please choose the device you want
to use.
2. Second, please push the buttons/slide the faders in the order you want to
get as `sensor_msgs/Joy`.
The script maps those buttons to `sensor_msgs/Joy/axes`
(and `sensor_msgs/Joy/buttons` if possible).
3. Please type `q` to quit. And the configure will be saved into
`/tmp/midi.yaml`.

### [`midi_write.py`](scripts/midi_write.py)
In order to control LEDs and active faders, need to output some MIDI commands
from you computer.
The script `midi_write.py` helps to dig the command and you can save
the configuration to a yaml file by `-w` option.

### [`midi_config_player.py`](scripts/midi_config_player.py)
This script publishes `sensor_msgs/Joy` to `/joy` based on a yaml file
configured by `interactive_midi_config.py` and `midi_write.py`.

[`configs`](configs) directory includes some yaml files for several MIDI
devices.

