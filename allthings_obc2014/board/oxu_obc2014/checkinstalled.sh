#!/bin/sh

echo "Check if CANDY is installed for auto-start: (should see candy below)"
echo "--------------------------------------------------------------------"
cat /etc/rc.local | grep candy
echo "--------------------------------------------------------------------"
