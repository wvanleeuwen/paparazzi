


all:	candy popcorn

candy:	serial.c candy.c
	gcc -static serial.c candy.c -o candy

popcorn: chdk_pipe.c
	gcc -static chdk_pipe.c -o popcorn

clean:
	rm -rf ./candy ./popcorn
