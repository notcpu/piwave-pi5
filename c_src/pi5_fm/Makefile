# Pi 5 native FM research tool
CC ?= gcc
CFLAGS ?= -O2 -Wall -Wextra -std=c11
LDLIBS = -lsndfile -lm

all: pi5_fm_rds

pi5_fm_rds: pi5_fm.c
	$(CC) $(CFLAGS) -o $@ $< $(LDLIBS)

clean:
	rm -f pi5_fm_rds
