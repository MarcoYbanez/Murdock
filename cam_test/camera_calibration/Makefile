INCLUDE_DIRS =  -I/usr/local/include/opencv4
#INCLUDE_DIRS =  -I/usr/include/opencv4
LIB_DIRS = 
export CC=g++ $(pkg-config --cflags --libs opencv4)
CDEFS=
CFLAGS= -O0 -g $(INCLUDE_DIRS) $(CDEFS)
LIBS= -lrt
CPPLIBS= -L/usr/local/lib/ -lopencv_core -lopencv_flann -lopencv_video -lopencv_highgui -lopencv_ccalib

HFILES= 
CFILES= 

SRCS= ${HFILES} ${CFILES}

all: calibrate temp web

clean:
	-rm -f *.o *.d
	-rm temp 
	-rm calibrate
	-rm web 

distclean:
	-rm -f *.o *.d

#disparity_map: disparity_map.o
#	$(CC) $(LDFLAGS) $(CFLAGS) $(INCLUDE_DIRS) -o $@ $@.o `pkg-config --libs opencv4` $(CPPLIBS)

temp: temp.o
	$(CC) $(LDFLAGS) $(CFLAGS) $(INCLUDE_DIRS) -o $@ $@.o `pkg-config --libs opencv4` $(CPPLIBS)

calibrate: calibrate.o
	$(CC) $(LDFLAGS) $(CFLAGS) $(INCLUDE_DIRS) -o $@ $@.o `pkg-config --libs opencv4` $(CPPLIBS)

web: web.o
	$(CC) $(LDFLAGS) $(CFLAGS) $(INCLUDE_DIRS) -o $@ $@.o `pkg-config --libs opencv4` $(CPPLIBS)

depend:

.c.o:
	$(CC) $(CFLAGS) -c $<

.cpp.o:
	$(CC) $(CFLAGS) -c $<
