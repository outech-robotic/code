#!/bin/bash

sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 type can loopback off
sudo ip link set can0 type can restart-ms 50
sudo ip link set can0 up
