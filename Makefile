CFLAG = -O0
INCLUDE_PATH = -I"/usr/src/kernels/4.18.0-193.14.2.el8_2.x86_64/include" -I"/usr/src/kernels/4.18.0-193.14.2.el8_2.x86_64/arch/x86/include/"
CC=gcc
PWD=$(shell pwd)
sraidk-objs := sraid.o
obj-m+=sraidk.o
all:
	make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) modules
clean:
	make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) clean
	rm -f *.ko
