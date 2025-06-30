#!/bin/bash

source ~/.bashrc

gnome-terminal \
    --tab --title="GNSS Read"       --command="bash -c 'sleep 1;rosrun gps_pkg gnss_read; exec bash'" \
    --tab --title="Set Origin"      --command="bash -c 'sleep 1.5;rosrun gps_pkg set_gnss_origin; exec bash'" \
    --tab --title="ENU Conversion"  --command="bash -c 'sleep 2;rosrun gps_pkg gnss_enu_convert; exec bash'"

