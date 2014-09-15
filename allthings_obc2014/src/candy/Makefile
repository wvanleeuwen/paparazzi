

CFLAGS = -static
LINK   = -lpthread
CFILES = serial.c \
	 chdk_pipe.c \
	 ../lib/udp/socket.c \
	 protocol.c
HFILES = $(CFILES:.c=.h)


all:	candy

candy:	$(CFILES) $(HFILES) candy.c
	gcc $(CFLAGS) $(CFILES) candy.c -o candy $(LINK)

clean:
	rm -rf ./candy
