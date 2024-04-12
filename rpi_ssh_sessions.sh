#!/bin/bash

# Replace <raspberry_pi_ip_address> with the actual IP address of your Raspberry Pi
RPI_IP="192.168.204.115"

# Replace <your_laptop_ip_address> with the actual IP address of your laptop
LAPTOP_IP="192.168.204.158"

# Open the first terminal
gnome-terminal -- bash -c "ssh  arno@$RPI_IP; wavemon; exec bash" -y

# Open the second terminal
gnome-terminal -- bash -c "ssh  arno@$RPI_IP; gst-launch-1.0 -v v4l2src device=/dev/video0 ! videoconvert ! videoscale ! video/x-raw,width=320,height=240 ! jpegenc ! rtpjpegpay ! udpsink host=$LAPTOP_IP port=5000 buffer-size=524288; exec bash" -y

# Open the third terminal
gnome-terminal -- bash -c "ssh  arno@$RPI_IP; exec bash" -y

