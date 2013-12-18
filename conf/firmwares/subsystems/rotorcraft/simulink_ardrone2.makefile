


$(TARGET).CFLAGS += -DSTABILIZATION_ATTITUDE_TYPE_INT
$(TARGET).CFLAGS += -DSTABILIZATION_ATTITUDE_TYPE_H=\"stabilization/stabilization_attitude_quat_int.h\"

$(TARGET).CFLAGS += -DAHRS_TYPE_H=\"boards/ardrone/simulink.h\"
$(TARGET).CFLAGS += -DINS_TYPE_H=\"boards/ardrone/simulink.h\"

$(TARGET).srcs += $(SRC_BOARD)/simulink.c



$(TARGET).srcs += $(SRC_BOARD)/paparazzi1.c
