# Out-of-tree gs_usb for Tegra kernels that ship without CONFIG_CAN_GS_USB.
# Build on Jetson: make && sudo make install. Do not commit *.ko artifacts.
obj-m += gs_usb.o

KDIR ?= /lib/modules/$(shell uname -r)/build
PWD  := $(shell pwd)

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install
	depmod -a
