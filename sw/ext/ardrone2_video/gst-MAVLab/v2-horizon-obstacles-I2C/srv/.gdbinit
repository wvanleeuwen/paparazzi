# This gdb init script automatically connects to the target and
# loads the program 
#
# $Id: .gdbinit,v 1.2 2006/01/22 17:08:05 strubi Exp $
# 

file srv1.bin
target remote :2000

set remotetimeout 999999
set remoteaddresssize 32

set prompt (bfin-jtag-gdb)\ 
source ../bin/scripts/mmr.gdb

define init
	monitor reset
	echo initialize memory...\n
	set *$EBIU_SDRRC = 0x0817
	set *$EBIU_SDBCTL = 0x0013
	set *$EBIU_SDGCTL = 0x0091998d
	echo ok.\n
	load srv1.bin
end

init

define syscr
	print/x * ((unsigned short *) 0xffc00104)
end

set scheduler-locking off
