###############################################################################
# CS/CNS 171 Fall 2015
#
# This is a template Makefile for the assignment. Edit it however you find
# convenient.
###############################################################################
CC = g++
FLAGS = -g -Wall -Werror -std=c++14
SOURCES = part1.o

EXENAME = my_prog

all: $(SOURCES)
	$(CC) $(FLAGS) $^ -o $(EXENAME) 

clean:
	rm -f *.o $(EXENAME)

.PHONY: all clean

