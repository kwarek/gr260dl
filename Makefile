CFLAGS := -O2 -W -Wall -ggdb
#LIBS := -lusb-1.0
PROG := gr260dl

all: ${PROG}

${PROG}: gr260dl.c
	gcc ${CFLAGS} ${LIBS} -o $@ $<
