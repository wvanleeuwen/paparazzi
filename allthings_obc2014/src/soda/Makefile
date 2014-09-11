
LIBJPEG = /usr/local/lib/libjpeg.a




all:	soda

$(LIBJPEG):
	tar -xvf ./jpegsrc.v9a.tar.gz
	cd ./jpeg-9a; ./configure
	make -C ./jpeg-9a
	sudo make -C ./jpeg-9a install

soda:	$(LIBJPEG) soda.c rgb2hsv.c jpeg_decode.c jpeg_decode.h
	g++ -static jpeg_decode.c soda.c rgb2hsv.c $(LIBJPEG) -o soda

clean:
	rm -rf ./soda 
