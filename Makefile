SOURCES=novena-scope.c gpio.c eim.c ddr3.c userspace.c \
	ad9520.c adc08d1020.c dac101c085.c lmh6518.c
OBJECTS=$(SOURCES:.c=.o)

SOURCES_devmem2=devmem2.c
OBJECTS_devmem2=$(SOURCES_devmem2:.c=.o)

SOURCES_cheezprint=chee-z-print.c
OBJECTS_cheezprint=$(SOURCES_cheezprint:.c=.o)

SOURCES_generate=generate.c
OBJECTS_generate=$(SOURCES_generate:.c=.o)

EXEC=novena-scope
MY_CFLAGS += -Wall -O0 -g `pkg-config libnl-3.0 --cflags` `pkg-config libnl-genl-3.0 --cflags`
MY_LIBS += `pkg-config libnl-3.0 --libs` `pkg-config libnl-genl-3.0 --libs`

all: $(OBJECTS) $(OBJECTS_devmem2) $(OBJECTS_cheezprint) $(OBJECTS_generate)
	$(CC) $(LIBS) $(LDFLAGS) $(OBJECTS) $(MY_LIBS) -o $(EXEC)
	$(CC) $(LIBS) $(LDFLAGS) $(OBJECTS_devmem2) $(MY_LIBS) -o devmem2
	$(CC) $(LIBS) $(LDFLAGS) $(OBJECTS_cheezprint) $(MY_LIBS) -o chee-z-print
	$(CC) $(LIBS) $(LDFLAGS) $(OBJECTS_generate) $(MY_LIBS) -o generate

clean:
	rm -f $(EXEC) $(OBJECTS)

.c.o:
	$(CC) -c $(CFLAGS) $(MY_CFLAGS) $< -o $@

