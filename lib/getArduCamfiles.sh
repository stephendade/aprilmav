#!/bin/bash

wget https://raw.githubusercontent.com/ArduCAM/MIPI_Camera/master/RPI/python/arducam_mipicamera.py
wget https://github.com/ArduCAM/MIPI_Camera/raw/master/RPI/lib/libarducam_mipicamera.so
sudo cp libarducam_mipicamera.so /usr/lib/
sudo ldconfig
