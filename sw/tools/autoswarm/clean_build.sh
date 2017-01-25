#!/bin/bash

for var in "$@"
do
make --no-print-directory -C /home/fred/paparazzi -f Makefile.ac AIRCRAFT=bebop2$var clean_ac &> /dev/null
make --no-print-directory -C /home/fred/paparazzi -f Makefile.ac AIRCRAFT=bebop2$var ap.compile 2>/dev/null
echo "Built: bebop 2$var"
done
