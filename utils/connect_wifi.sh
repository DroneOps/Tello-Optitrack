#!/bin/bash

#Script to connect to the Tello drone's WiFi network.
IFACE="wlan0"

# Load configuration from tello.conf
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
source "$DIR/tello.conf"

# Check if already connected to the desired network
CURRENT=$(nmcli -t -f active,ssid dev wifi | grep '^yes' | cut -d: -f2)

if [ "$CURRENT" == "$SSID" ]; then
    echo "Connected to $SSID"
    exit 0
fi

# if not connected, try to connect to the desired network
if [ -n "$PASSWORD" ]; then
    echo " Connecting to $SSID with password..."
    nmcli dev wifi connect "$SSID" password "$PASSWORD"
else
    echo " Connecting to $SSID without password..."
    nmcli dev wifi connect "$SSID"
fi

# Check if the connection was successful
if [ $? -eq 0 ]; then
    echo " Connected to $SSID"
else
    echo " Error connecting to $SSID"
    exit 1
fi
