#!/usr/bin/sh

sudo apt-get update
sudo apt-get -y upgrade
sudo apt-get -y dist-upgrade
sudo apt-get install -y build-essential python-dev python-setuptools python-pip python-smbus
sudo apt-get install -y libncursesw5-dev libgdbm-dev libc6-dev
sudo apt-get install -y zlib1g-dev libsqlite3-dev tk-dev
sudo apt-get install -y libssl-dev openssl
sudo apt-get install -y libffi-dev

wget https://www.python.org/ftp/python/3.7.9/Python-3.7.9.tgz
tar xvfz Python-3.7.9.tgz
cd Python-3.7.9
./configure
make
sudo make install
