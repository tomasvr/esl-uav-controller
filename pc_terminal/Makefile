
CC=gcc
CFLAGS = -g -Wall -lm
EXEC = ./pc-terminal 
C_SOURCE_FILES += \
$(abspath ../states.c) \

all:
	$(CC) $(CFLAGS) pc_terminal.c $(C_SOURCE_FILES) -o $(EXEC)

run: all
	$(EXEC)