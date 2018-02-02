CC = gcc
OBJS = PicoFlash.o

all: PicoFlash

PicoFlash: $(OBJS)

clean:
	rm $(OBJS) PicoFlash

.PHONY: all clean
