SOURCES=novena-scope.c gpio.c eim.c ad9520.c adc08d1020.c ddr3.c userspace.c
OBJECTS=$(SOURCES:.c=.o)
EXEC=novena-scope
MY_CFLAGS += -Wall -O0 -g `pkg-config libnl-3.0 --cflags` `pkg-config libnl-genl-3.0 --cflags`
MY_LIBS += `pkg-config libnl-3.0 --libs` `pkg-config libnl-genl-3.0 --libs`
#MY_CFLAGS += -Wall -O0 -g
#MY_LIBS +=

all: $(OBJECTS)
	$(CC) $(LIBS) $(LDFLAGS) $(OBJECTS) $(MY_LIBS) -o $(EXEC)

clean:
	rm -f $(EXEC) $(OBJECTS)

.c.o:
	$(CC) -c $(CFLAGS) $(MY_CFLAGS) $< -o $@

