
obj-$(CONFIG_DENSO_HW_CTRL) += denso_hw_ctrl.o
#ccflags-y := -Wall -Werror -DDEBUG

denso_hw_ctrl-objs := src/mtd_api.o src/nor_api.o src/hw_ctrl_module.o

# All of the below was for cross compiling
#export ARCH:=arm
#export CROSS_COMPILE_DIR=~/repos/honda-denso/prebuilts/gcc/linux-x86/arm/arm-eabi-4.8/bin/
#export PATH=$PATH:$CROSS_COMPILE_DIR
#export CROSS_COMPILE=$CROSS_COMPILE_DIR/arm-eabi-
#KDIR := /home/robert/repos/honda-denso/kernel/android-3.14/
#PWD := $(shell pwd)
#default:
#	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules
#clean:
#	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) clean

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules


clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
