# joy_mouse

## setup
`/dev/input/mouse[0-9]*` requires super user permission in default of ubuntu.
So we need to chane it.

Please open your favorite editor and make a file at `/etc/udev/rules.d/99-input.rules`
with following content:
```
SUBSYSTEM=="input", MODE="666"
```

And reboot your computer or re-plug your mouse.
