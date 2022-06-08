###############################################################################
# CS/CNS 171 Fall 2015
#
# This is a template Makefile for the assignment. Edit it however you find
# convenient.
###############################################################################
CC = g++
FLAGS = -g -Wall -Werror -std=c++14

# The following line is a relative directory reference that assumes the Eigen
# folder--which your program will depend on--is located one directory above the
# directory that contains this Makefile.
INCLUDE = -isystem../
SOURCES = wireframe.cpp helper.cpp
SOURCES_EC = wireframe_ec.cpp helper.cpp

EXENAME = wireframe
EXENAME_EC = wireframe_ec

all: wireframe wireframe_ec

wireframe: 
	$(CC) $(FLAGS) -o $(EXENAME) $(INCLUDE) $(SOURCES)

wireframe_ec: 
	$(CC) $(FLAGS) -o $(EXENAME_EC) $(INCLUDE) $(SOURCES_EC)

clean:
	rm -f *.o $(EXENAME) $(EXENAME_EC)

.PHONY: all clean

