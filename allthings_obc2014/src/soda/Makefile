


all:	soda

soda:	soda.c rgb2hsv.c jpeg_decode.c jpeg_decode.h
	g++ -static jpeg_decode.c soda.c rgb2hsv.c /usr/local/lib/libjpeg.a -o soda

clean:
	rm -rf ./soda 
