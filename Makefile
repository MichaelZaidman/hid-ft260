KRELEASE ?= $(shell uname -r)
KBUILD ?= /lib/modules/$(KRELEASE)/build
obj-m += hid-ft260.o

all:
	$(MAKE) -C $(KBUILD) M=$(shell pwd) modules

clean:
	$(MAKE) -C $(KBUILD) M=$(shell pwd) clean
