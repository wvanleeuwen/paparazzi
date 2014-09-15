
LIBJPEG = /usr/local/lib/libjpeg.a


CFLAGS = -static
CFILES = \
	rgb2hsv.c \
	jpeg_decode.c \
	jpeg_encode.c \
	color_probs.c \
	integral_image.c \
	../lib/udp/socket.c
HFILES = $(CFILES:.c=.h )

all:	soda makeheaders

$(LIBJPEG):
	tar -xvf ./jpegsrc.v9a.tar.gz
	cd ./jpeg-9a; ./configure
	make -C ./jpeg-9a
	sudo make -C ./jpeg-9a install

soda:	soda.c $(LIBJPEG) $(CFILES) $(HFILES)
	g++ $(CFLAGS) $(CFILES) soda.c $(LIBJPEG) -o soda

clean:
	rm -rf ./soda ./makeheaders

makeheaders: soda.c $(LIBJPEG) $(CFILES) $(HFILES)
	g++ -DMAKE_HEADERS $(CFLAGS) $(CFILES) soda.c $(LIBJPEG) -o makeheaders

explain:
	echo "$(CFILES) $(HFILES)"
