#
# Makefile
# gcc mbm.c -o mbm -lmodbus
#

RM = \rm -f
PROG = mbm
BINPATH=/usr/local/bin

# DEBUG	= -O2
CC	= gcc
INCLUDE	= -I.
CFLAGS	= $(DEBUG) $(INCLUDE) -Wformat=2 -Wall -Winline  -pipe -fPIC 

LSWI = -L
LIBS =  $(LSWI)/usr/local/lib

# List of objects files for the dependency
OBJS_DEPEND= -lmodbus

# Source files
SRC	= $(PROG).c

# OPTIONS = --verbose

all: target

target: Makefile
	@echo "--- Compile and Linking all object files to create the whole file: $(PROG) ---"
	$(CC) $(SRC) -o $(PROG) $(CFLAGS) $(OBJS_DEPEND) $(OPTIONS)
	@echo ""

clean :
	@echo "---- Cleaning all object files in all the directories ----"
	$(RM) $(PROG)
	@echo "" 

install : target
	@echo "---- Install binaries ----"
	cp $(PROG) $(BINPATH)