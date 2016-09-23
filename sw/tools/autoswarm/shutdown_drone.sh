#!/bin/bash

for var in "$@"
do
if(({ echo "sh /bin/ardrone3_shutdown.sh"; sleep 3; } | telnet 192.168.40.$var) | grep "Connection closed by foreign host")
then
echo "Shutdown bebop2$var"
else
echo "FAIL bebop2$var"
fi
done
