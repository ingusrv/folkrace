#!/bin/bash

if [ -f /etc/systemd/user/folkrace-robot.service ]; then
    # vispirms nodrošināmies, ka pakalpojums nestrādā
    systemctl --user stop folkrace-robot.service 
fi
sudo rm /etc/systemd/user/folkrace-robot.service
echo "pakalpojums atinstalēts"
