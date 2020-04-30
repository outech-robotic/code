# How to use


Put the content of the `config` folder in `/etc/default/`

Put `isotp@.service` in `/etc/systemd/system/isotp@.service`

Put `can0` in `/etc/network/interfaces.d/can0`


Run
```
sudo systemctl enable isotp@motor_board
sudo systemctl enable isotp@servo_board_0
sudo systemctl enable isotp@servo_board_1
sudo systemctl enable isotp@servo_board_2
sudo systemctl enable isotp@servo_board_3
sudo systemctl enable isotp@servo_board_4
sudo systemctl enable isotp@servo_board_5
sudo systemctl enable isotp@servo_board_6
sudo systemctl enable isotp@servo_board_7
sudo systemctl enable isotp@servo_board_8
sudo systemctl enable isotp@sensor_board_0
```

**Restart.**

All the isotp services will launch and the CAN interface will be brought up.
