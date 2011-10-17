gluttest : pprzopengl.c
	gcc -g -O2 -Wall `pkg-config --cflags --libs gtk+-2.0 gtkglext-1.0 gtkglext-x11-1.0 ` -g -o pprzopengl  pprzopengl.c `pkg-config gtk+-2.0 gtkglext-1.0 gtkglext-x11-1.0 --libs` -s `pcre-config --libs` -lglibivy
clean : 
	rm -rf pprzopengl
