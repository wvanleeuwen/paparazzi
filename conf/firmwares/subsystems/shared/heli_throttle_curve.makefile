#  Throttle Curve
#
#

$(TARGET).CFLAGS += -DUSE_THROTTLE_CURVE=1
$(TARGET).srcs   += modules/helicopter/throttle_curve.c

