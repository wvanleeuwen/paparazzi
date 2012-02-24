# Hey Emacs, this is a -*- makefile -*-

include $(PAPARAZZI_SRC)/conf/boards/lisa_m_1.0.makefile

#
# Overrule board version
#
BOARD_VERSION = 2.0

#
# Swap GPS UART with spektrum UART
#
RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT   = UART1
GPS_PORT = UART3

