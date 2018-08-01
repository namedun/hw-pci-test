
APP_NAME = hw_test

CC      = gcc
AR      = ar
LD      = ld
CFLAGS  = -c -O2 -Wall
LDFLAGS = -o $(APP_NAME) -lpthread

INSTALL_DIR ?=

SOURCES=hw_test.c \
        gnu_getopt/gnu_getopt.c \
        gnu_getopt/gnu_getopt_long.c

OBJECTS=$(SOURCES:.c=.o)

build: $(OBJECTS)
	$(CC) $(OBJECTS) $(LDFLAGS)

.c.o:
	$(CC) $(CFLAGS) $< -o $@

clean:
	-@rm -rf $(APP_NAME) *.o
