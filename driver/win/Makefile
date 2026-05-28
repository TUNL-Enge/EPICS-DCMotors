# Makefile for ETH32 API on Windows
CC=gcc
CFLAGS=-D_REENTRANT -D_GNU_SOURCE -DWINDOWS -mthreads -Wall -O2 -I ../dependencies/WpdPack/Include
# Location of Microsoft Visual C++ LIB command line tool
# (Not required to just compile the API)
MSLIB=C:\MSDEVSTD\BIN\LIB.EXE

DIR=../

SRC=$(addprefix $(DIR), eth32.c eth32cfg.c iphelper.c eth32cfg_pcap.c threads.c eth32_internal.c dbllist.c devtable.c socket.c)

OBJ = $(notdir $(SRC:.c=.o))

OUTNAME=eth32api

# Default way to compile .c into .o
%.o: $(DIR)%.c
	$(CC) $(CFLAGS) -c $^


all: $(OUTNAME).dll
libfile: $(OUTNAME).lib

$(OUTNAME).dll: $(OBJ) exports-mingw.def
	gcc -shared -Wl,--add-stdcall-alias -o $@ $(OBJ) exports-mingw.def -lws2_32
	strip -s $@

$(OUTNAME).lib: exports-msimplib.def
	$(MSLIB) /MACHINE:X86 /def:$^ /out:$@

clean:
	del $(OUTNAME).dll $(OBJ)
