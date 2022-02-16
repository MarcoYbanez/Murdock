
INCLUDE_DIRS = 
LIB_DIRS = 
CC=g++

CDEFS=
CFLAGS= 
LIBS= 

HFILES= 
CFILES= monitor_time.cpp tracking.cpp Qt.c

SRCS= ${HFILES} ${CFILES}
OBJS= ${CFILES:.c=.o}

all: monitor_time Qt tracking

clean:
	-rm -f *.o *.d
	-rm -f monitor_time Qt tracking 

monitor_time: monitor_time.o
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $@.o 

Qt: Qt.o
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $@.o 

tracking: tracking.o
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $@.o 

depend:

.c.o:
	$(CC) $(CFLAGS) -c $<
