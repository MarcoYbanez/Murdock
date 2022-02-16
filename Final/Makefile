INCLUDE_DIRS =  -I/usr/local/include/opencv4 -lpthread
#INCLUDE_DIRS =  -I/usr/include/opencv4
LIB_DIRS = 
export CC=g++   $(pkg-config --cflags --libs opencv4)
CDEFS=
CFLAGS= -O0 -g $(INCLUDE_DIRS) $(CDEFS)
LIBS= -lrt
CPPLIBS= -L/usr/local/lib/ -lopencv_core -lopencv_flann -lopencv_video -lopencv_highgui -lopencv_ccalib -lpthread

HFILES= 
CFILES= 

SRCS= ${HFILES} ${CFILES}

all: main

clean:
	-rm -f *.o *.d
	-rm main

distclean:
	-rm -f *.o *.d

main: main.o
	$(CC) $(LDFLAGS) $(CFLAGS) $(INCLUDE_DIRS) -o $@ $@.o `pkg-config --libs opencv4` $(CPPLIBS)

depend:

.c.o:
	$(CC) $(CFLAGS) -c $<

.cpp.o:
	$(CC) $(CFLAGS) -c $<
