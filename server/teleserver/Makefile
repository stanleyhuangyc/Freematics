CC=gcc
CFLAGS=-O3 -Wunused-result
OBJS=teleserver.o httppil.o httpd.o httpjson.o udpserver.o
HEADERS=teleserver.h httpint.h httpapi.h
TARGET=teleserver

ifndef DEBUG
DFLAGS += -s
else
DEFINES+= -DHTTP_DEBUG
LDFLAGS += -g
endif

%.o: %.c $(HEADERS)
	$(CC) $(DEFINES) -c -o $@ $(CFLAGS) $(filter %.c, $^)

all: $(OBJS)
	$(CC) $(LDFLAGS) $(OBJS) -o $(TARGET)

clean:
	@rm -f $(TARGET)
	@rm -f *.o
