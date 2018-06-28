
CFLAGS = -Wall
CC = gcc
LIBS = -lcanlib -lpthread -lm -lutil

INCLUDE = kvalt_logger.c kvaser.c stp.c data_processor.c

all:
	@echo ***** Building Kvaser/Altera CAN logger *****
	rm -rf build
	mkdir build
	$(CC) $(CFLAGS) -o build/kvalt_logger $(INCLUDE) $(LIBS)

clean:  
	@echo ***** Cleaning Kvaser/Altera CAN logger *****
	rm -rf build


