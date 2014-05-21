set target-async on
set mem inaccessible-by-default off
tar ext /dev/ttyACM0
mon version
mon swdp_scan
att 1
