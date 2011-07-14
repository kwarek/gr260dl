#LIBS := -lusb-1.0
PROG := gr260dl

all: ${PROG}

${PROG}: gr260dl.c
	gcc -O2 -W -Wall ${LIBS} -o $@ $<
