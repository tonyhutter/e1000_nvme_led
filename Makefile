obj-m += pciehp_craye1k.o

# Point this to your Linux kernel source directory
UNAME_R := $(shell uname -r)
KDIR:=/usr/src/kernels/$(UNAME_R)

all:
	make -C $(KDIR) M=$(PWD) modules

clean:
	make -C $(KDIR) M=$(PWD) clean
