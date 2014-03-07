# jsk_teleop_joy package

## Introduction
This package provides physical UI for teleoperatoin for example joy sticks,
game controllers and midi instruments.

## Supported game controllers
This package supports following game controllers:

1. XBox360 controller
2. PS3 Controller (Bluetooth)
3. PS3 Controller (USB)

In order to use PS3 controller via Bluetooth,
see [this instruction](http://wiki.ros.org/ps3joy/Tutorials/PairingJoystickAndBluetoothDongle).

## Architecture of jsk\_teleop\_joy
jsk\_teleop\_joy (the main script is [`joy_main.py`](scripts/joy_main.py)) is based on plugin architecture.

## MIDI controllers
### [`interactive_midi_config.py`](scripts/interactive_midi_config.py)
You can configure MIDI devices by this script interactively. [movie](http://www.youtube.com/watch?v=1JOKra7gZVs)

0. Connect you MIDI device.
1. First, the script asks the name of device, please choose the device you want to use.
2. Second, please push the buttons/slide the faders in the order you want to get as `sensor_msgs/Joy`.
The script maps those buttons to `sensor_msgs/Joy/axes` (and `sensor_msgs/Joy/buttons` if possible).
3. Please type `q` to quit. And the configure will be saved into `/tmp/midi.yaml`.

### [`midi_write.py`](scripts/midi_write.py)
In order to control LEDs and active faders, need to output some MIDI commands from you computer.
The script `midi_write.py` helps to dig the command and you can save the configuration to a yaml file by `-w` option.

### [`midi_config_player.py`](scripts/midi_config_player.py)
This script publishes `sensor_msgs/Joy` to `/joy` based on a yaml file configured by
`interactive_midi_config.py` and `midi_write.py`.

