#!/usr/bin/sh

sudo gpasswd --add keti dialout
sudo gpasswd --add keti tty
sudo systemctl stop nvgetty.service
sudo systemctl disable nvgetty.service

sudo mkdir /etc/mavlink-router
sudo cp main.conf /etc/mavlink-router

cp runUDPMav.js /home/keti/
pm2 start /home/keti/runUDPMav.js --name MavUDP
pm2 save
