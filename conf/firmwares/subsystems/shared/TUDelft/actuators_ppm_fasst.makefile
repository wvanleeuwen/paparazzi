# for Tiny v2 or Twog v1


fbw.CFLAGS += -DACTUATORS=\"./TUDelft/servos_ppm_hw_fasst.h\" -DSERVOS_PPM_MAT
fbw.srcs += $(SRC_ARCH)/TUDelft/servos_ppm_hw_fasst.c actuators.c
