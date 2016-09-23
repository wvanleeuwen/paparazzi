#!/bin/bash

BASE="airframe_base.xml"
for var in "$@"
do
cp "/home/fred/paparazzi/conf/airframes/TUDELFT/autoswarm/$BASE" "/home/fred/paparazzi/conf/airframes/TUDELFT/autoswarm/bebop2$var.xml"
sed -i s/192.168.40.30/192.168.40.$var/g "/home/fred/paparazzi/conf/airframes/TUDELFT/autoswarm/bebop2$var.xml"
done
