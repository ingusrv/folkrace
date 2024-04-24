#!/bin/bash

address=$1
port=$2
token=$3

cat > ./folkrace-robot.service << EOF
[Unit]
Description=folkrace robota pakalpojums
After=network.target
StartLimitIntervalSec=0

[Service]
Type=simple
ExecStart=/usr/bin/python3 $(pwd)/robot.py -a $address -p $port -t $token 

[Install]
WantedBy=multi-user.target
EOF

if [ -f /etc/systemd/user/folkrace-robot.service ]; then
    echo "pakalpojums jau eksistē, atjaunināt? (y/n)"
    read choice
    if [ $choice == 'y' ] || [ $choice == 'Y' ]; then
        sudo mv folkrace-robot.service /etc/systemd/user/
        systemctl --user daemon-reload
        echo "pakalpojums instalēts kā 'folkrace-robot.service'"
        echo "izmantojat 'systemctl --user start folkrace-robot' lai startētu"
    else
        echo "instalācija atcelta"
    fi
else
        sudo mv folkrace-robot.service /etc/systemd/user/
        echo "pakalpojums instalēts kā 'folkrace-robot.service'"
        echo "izmantojat 'systemctl --user start folkrace-robot' lai startētu"
fi


