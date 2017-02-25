#!/bin/bash

# grant acces i2c
sudo chmod a+rw /dev/i2c-*

# load pigpio daemon
#sudo killall pigpiod
#sudo pigpiod



# Synchronize time with master RP
# http://askubuntu.com/questions/488072/setting-up-a-standalone-ntp-server-on-ubuntu
#sudo service ntp restart
#sudo ntpdate -s 192.168.0.210