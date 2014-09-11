


all:	candy

candy:	serial.c chdk_pipe.c socket.c protocol.c candy.c
	gcc -static serial.c chdk_pipe.c socket.c protocol.c candy.c -o candy -lpthread

clean:
	rm -rf ./candy ./popcorn
