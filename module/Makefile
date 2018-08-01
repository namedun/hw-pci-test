
KERNELDIR=/usr/src/linux-headers-$(shell uname -r)
PWD=$(shell pwd)

obj-m := hw_drv.o
hw_drv-objs := hw_drv_src.o

modules:
	$(MAKE) -j 4 -C $(KERNELDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) clean
