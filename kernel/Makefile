obj-m := kosagi-fpga-kernel.o
KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)
SYM=$(PWD)

all:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) clean
