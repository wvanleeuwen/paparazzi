#
# Expected from board file or overriden as xml param :
#
#


ap.CFLAGS += -DDOWNLINK -DDOWNLINK_DEVICE=Wifi
ap.CFLAGS += -DDOWNLINK_TRANSPORT=WifiTransport
ap.srcs += subsystems/datalink/downlink.c subsystems/datalink/wifi.c
ap.srcs += $(SRC_FIRMWARE)/datalink.c $(SRC_FIRMWARE)/telemetry.c
