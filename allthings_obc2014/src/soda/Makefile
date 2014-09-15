
LIBJPEG = /usr/local/lib/libjpeg.a


CFLAGS = -static
CFILES = \
	rgb2hsv.c \
	jpeg_decode.c \
	color_probs.c \
	integral_image.c \
	../lib/udp/socket.c
HFILES = $(CFILES:.c=.h )

all:	soda

$(LIBJPEG):
	tar -xvf ./jpegsrc.v9a.tar.gz
	cd ./jpeg-9a; ./configure
	make -C ./jpeg-9a
	sudo make -C ./jpeg-9a install

soda:	$(LIBJPEG) $(CFILES) $(HFILES)
	g++ $(CFLAGS) $(CFILES) soda.c $(LIBJPEG) -o soda

clean:
	rm -rf ./soda

explain:
	echo "$(CFILES) $(HFILES)"
