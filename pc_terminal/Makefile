
CC=gcc
CFLAGS = -g -Wall -lm
EXEC = ./pc-terminal --normal /dev/input/js0

all:
	$(CC) $(CFLAGS) pc_terminal.c -o $(EXEC)

run: all
	$(EXEC)