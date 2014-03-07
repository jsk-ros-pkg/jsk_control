# jsk_teleop_joy package

## introduction
This package provides physical UI for teleoperatoin for example joy sticks,
game controllers and midi instruments.

## Supported Game controllers
This package supports following game controllers:

1. XBox360 controller
2. PS3 Controller (Bluetooth)
3. PS3 Controller (USB)

In order to use PS3 controller via Bluetooth,
see [this instruction](http://wiki.ros.org/ps3joy/Tutorials/PairingJoystickAndBluetoothDongle).


## MIDI controllers
### `interactive_midi_config.py`
You can configure MIDI devices by this script interactively.

0. Connect you MIDI device.
1. First, the script asks the name of device, please choose the device you want to use.
2. Second, please push the buttons/slide the faders in the order you want to get as `sensor_msgs/Joy`.
The script maps those buttons to `sensor_msgs/Joy/axes` (and `sensor_msgs/Joy/buttons` if possible).
3. Please type `q` to quit. And the configure will be saved into `/tmp/midi.yaml`.

### `midi_write.py`
LEDやアクティブフェーダなどを制御するには、MIDIのoutputを計算機から叩く必要がある。そのための便利スクリプト。
`-w`オプションを利用することで、yamlファイルに追記することができる。

### `midi_config_player.py`
This script publishes `sensor_msgs/Joy` to `/joy` based on a yaml file configured by
`interactive_midi_config.py` and `midi_write.py`.

