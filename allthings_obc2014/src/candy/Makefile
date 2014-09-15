


all:	candy

candy:	serial.c chdk_pipe.c ../lib/udp/socket.c protocol.c candy.c
	gcc -static serial.c chdk_pipe.c ../lib/udp/socket.c protocol.c candy.c -o candy -lpthread

clean:
	rm -rf ./candy
