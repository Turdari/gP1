#
# Makefile for the mcc device driver.
#
# Note! Dependencies are done automagically by 'make dep', which also
# removes any old dependencies. DON'T put your own dependencies here
# unless it's something special (ie not a .c file).
#
# Note 2! The CFLAGS definition is now in the main makefile...

O_TARGET := mccdrv.o
MCC_OBJS := mcc.o device.o

MOD_LIST_NAME := MCC_MODULES
M_OBJS := mccdrv.o

O_OBJS := $(MCC_OBJS)
OX_OBJS := $(MCC_OBJS)

include $(TOPDIR)/Rules.make

tar:
		tar -cvf /dev/f1 .