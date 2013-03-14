#!/bin/bash

# add respositories
sudo apt-add-repository ppa:sat-metalab/metalab
sudo apt-add-repository ppa:openrave/release
sudo apt-add-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
sudo apt-get update

# install deps
sudo apt-get -y install automake bison build-essential cmake-curses-gui flex libtool 
sudo apt-get -y install ffmpeg freeglut3-dev libavcodec-dev libavdevice-dev libavformat-dev libboost-filesystem-dev libboost-program-options-dev libboost-python-dev libboost-regex-dev libboost-system-dev libboost-thread-dev libcppintrospection-3.0-dev liblo-dev libopenscenegraph-dev libswscale-dev libxml++2.6-dev
sudo apt-get -y install libspatosc-dev
sudo apt-get -y install libspnav-dev spacenavd
sudo apt-get -y install python python-dev python-setuptools
sudo apt-get -y install libbullet-dev
sudo apt-get -y install libpoco-dev
sudo apt-get -y install ffmpeg
sudo apt-get -y install libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev
sudo apt-get -y install libshmdata-0.6-dev
sudo apt-get -y install libpcl-1.6-all
sudo apt-get -y install libtinyxml-dev

