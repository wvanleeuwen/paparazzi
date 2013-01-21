all:
	cp ./luftboot.c.fireswarm ../luftboot/src
	make -C ../
	cd ../luftboot/src; make flash BMP_PORT=/dev/ttyACM0 DEVICE_SERIAL=fsl-001; git checkout ./
