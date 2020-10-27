
CC=gcc
CFLAGS = -g -Wall -lm
EXEC = ./pc-terminal 
C_SOURCE_FILES += \
$(abspath ../common/states.c) \
$(abspath ../common/comm.c) \
$(abspath ./joystick.c) \

all:
	$(CC) $(CFLAGS) pc_terminal.c $(C_SOURCE_FILES) -o $(EXEC)

run: all
	$(EXEC)