#!/bin/bash
MAC="88:57:21:B6:50:56"
MESSAGE=$1

if [ -z "$MESSAGE" ]; then
    echo "Usage: ./send_ble.sh <message>"
    exit 1
fi

HEX_MESSAGE=$(echo -n "$MESSAGE" | xxd -p)
gatttool -b $MAC --char-write-req -a 0x002a -n $HEX_MESSAGE