#!/bin/bash

# Connects the system to the Tello drone WiFi network.
# Loads network configuration from config/tello.conf.

IFACE="wlan0"

# Load WiFi configuration from tello.conf
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
source "$DIR/../config/tello.conf"

# Check active connection
CURRENT=$(nmcli -t -f active,ssid dev wifi | grep '^yes' | cut -d: -f2)

if [ "$CURRENT" == "$SSID" ]; then
    echo "Connected to $SSID"
    exit 0
fi

# Connect to target network
if [ -n "$PASSWORD" ]; then
    echo " Connecting to $SSID with password..."
    nmcli dev wifi connect "$SSID" password "$PASSWORD"
else
    echo " Connecting to $SSID without password..."
    nmcli dev wifi connect "$SSID"
fi

# Check connection status
if [ $? -eq 0 ]; then
    echo " Connected to $SSID"
else
    echo " Error connecting to $SSID"
    exit 1
fi
