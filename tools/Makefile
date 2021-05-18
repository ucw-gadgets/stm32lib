PC=pkg-config
UCW_CFLAGS := $(shell $(PC) --cflags libucw)
UCW_LDFLAGS := $(shell $(PC) --libs libucw)

CFLAGS=$(UCW_CFLAGS) -O2 -std=gnu99 -Wall -Wextra -Wno-parentheses -Wstrict-prototypes -Wmissing-prototypes
LDFLAGS=$(UCW_LDFLAGS) -lz

all: dfu-sign

dfu-sign: dfu-sign.c

clean:
	rm -f dfu-sign *.o
