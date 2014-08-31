# Hey Emacs, this is a -*- makefile -*-

# InterMCU type CAN

ifneq ($(TARGET),sim)
$(TARGET).CFLAGS += -DINTER_MCU -DMCU_CAN_LINK
$(TARGET).srcs += ./link_mcu_can.c
endif

