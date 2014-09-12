#!/bin/sh

# Build Essential
sudo apt-get install build-essential

# GCS
sudo apt-get install gcc

# SVN client
sudo apt-get install subversion

# PV: To have progress while installing or uncompressing
sudo apt-get install pv

# LIBUSB
sudo apt-get install libusb-dev
# or sudo apt-get installlibusb-1.0-0 libusb-1.0-dev

# LUA
sudo apt-get install libreadline6-dev
sudo apt-get install lua5.2 lua5.2-devi

# NTP for time
sudo apt-get install ntpdate
ntpdate pool.ntp.org

