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
このスクリプトを使うと、対話的にmidiデバイスの入力を設定できる。

1. まず実行すると、デバイスの名前を聞かれるので、答えてください
2. 次に順番にボタンを押すと、その順番でボタンが`sensor_msgs/Joy`メッセージのaxes(および可能であればbutton)にマッピングされる。
3. `q`を押して終了すると、その設定が`/tmp/midi.yaml`に保存される。

### `midi_write.py`
LEDやアクティブフェーダなどを制御するには、MIDIのoutputを計算機から叩く必要がある。そのための便利スクリプト。
`-w`オプションを利用することで、yamlファイルに追記することができる。

### `midi_config_player.py`
`interactive_midi_config.py`および`midi_write.py`で生成したyamlファイルをもとに、`sensor_msgs/Joy`をpublishする。
