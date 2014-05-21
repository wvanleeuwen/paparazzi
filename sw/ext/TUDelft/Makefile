all:
	cp ./luftboot.c.fireswarm_2_0 ../luftboot/src/luftboot.c
	rm -rf ../luftboot/src/luftboot.bin ../liftboot/src/luftboot
	make -C ../ DEV_SERIAL=fsl-001 VERSION=V2.0
	cd ../luftboot/src; make flash BMP_PORT=/dev/ttyACM0; git checkout ./
