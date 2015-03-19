#############################################################################
# Makefile for building: Evolutionary Learning
#############################################################################

####### Compiler, tools and options

####### Build rules
TARGET			= avoidance
DEL_FILE      	= rm -f

all: 
	g++ -pipe -g -Wall -W -MMD -o $(TARGET) main.cpp `pkg-config opencv --cflags --libs`

clean:	
	$(DEL_FILE) $(TARGET) $(TARGET).d

