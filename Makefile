SOURCES=novena-scope.c gpio.c eim.c ad9520.c adc08d1020.c ddr3.c \
	userspace.c dac101c085.c lmh6518.c
SOURCES_devmem2=devmem2.c
OBJECTS=$(SOURCES:.c=.o)
OBJECTS_devmem2=$(SOURCES_devmem2:.c=.o)
EXEC=novena-scope
MY_CFLAGS += -Wall -O0 -g `pkg-config libnl-3.0 --cflags` `pkg-config libnl-genl-3.0 --cflags`
MY_LIBS += `pkg-config libnl-3.0 --libs` `pkg-config libnl-genl-3.0 --libs`

all: $(OBJECTS) $(OBJECTS_devmem2)
	$(CC) $(LIBS) $(LDFLAGS) $(OBJECTS) $(MY_LIBS) -o $(EXEC)
	$(CC) $(LIBS) $(LDFLAGS) $(OBJECTS_devmem2) $(MY_LIBS) -o devmem2

clean:
	rm -f $(EXEC) $(OBJECTS)

.c.o:
	$(CC) -c $(CFLAGS) $(MY_CFLAGS) $< -o $@

