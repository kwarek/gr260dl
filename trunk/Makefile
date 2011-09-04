################### program ###################
CFLAGS := -O2 -W -Wall -ggdb
#LIBS := -lusb-1.0
PROG := gr260dl

all: ${PROG}

${PROG}: gr260dl.c
	gcc ${CFLAGS} ${LIBS} -o $@ $<

install:
	install -g root -o root -m 755 ${PROG} /usr/bin/${PROG}

################### module ###################
OBJBASE := pl2303

obj-m += ${OBJBASE}.o

KVER := $(shell uname -r)
KDIR := /lib/modules/${KVER}/build

modules:
# CFLAGS has to be undefined
	make -C ${KDIR} M=${PWD} CFLAGS= PROG=

modules_install:
	make -C ${KDIR} M=${PWD} modules_install

modules_clean:
	@rm -vf Module.symvers built-in.o modules.order ${OBJBASE}.{ko,o,mod.c,mod.o}
