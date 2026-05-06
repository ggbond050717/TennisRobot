#!/bin/bash
echo 'KERNEL=="ttyUSB*",ATTRS{idVendor}=="1a86",ATTRS{idProduct}=="7523",MODE:="0666",GROUP:="dialout",SYMLINK+="imu"' >/etc/udev/rules.d/99-imu.rules