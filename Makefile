# SX1255 Library Makefile

CC = gcc
CFLAGS = -Wall -Wextra -O2 -fPIC
LDFLAGS = -lgpiod -lm

# Library files
LIB_NAME = libsx1255
STATIC_LIB = $(LIB_NAME).a
SHARED_LIB = $(LIB_NAME).so

# Source files
SOURCES = sx1255.c
OBJECTS = $(SOURCES:.c=.o)
HEADERS = sx1255.h

# Default target
all: $(STATIC_LIB) $(SHARED_LIB)

# Static library
$(STATIC_LIB): $(OBJECTS)
	ar rcs $@ $^

# Shared library
$(SHARED_LIB): $(OBJECTS)
	$(CC) -shared -o $@ $^ $(LDFLAGS)

# Object files
%.o: %.c $(HEADERS)
	$(CC) $(CFLAGS) -c $< -o $@

# Clean build artifacts
clean:
	rm -f $(OBJECTS) $(STATIC_LIB) $(SHARED_LIB)

# Install headers and libraries (optional)
install: $(STATIC_LIB) $(SHARED_LIB)
	sudo cp $(HEADERS) /usr/local/include/
	sudo cp $(STATIC_LIB) $(SHARED_LIB) /usr/local/lib/
	sudo ldconfig

# Uninstall
uninstall:
	sudo rm -f /usr/local/include/$(HEADERS)
	sudo rm -f /usr/local/lib/$(STATIC_LIB) /usr/local/lib/$(SHARED_LIB)
	sudo ldconfig

# Example program
example: example.c $(STATIC_LIB)
	$(CC) $(CFLAGS) -o $@ $< -L. -lsx1255 $(LDFLAGS)

.PHONY: all clean install uninstall example