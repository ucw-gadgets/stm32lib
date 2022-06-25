PC=pkg-config
UCW_CFLAGS := $(shell $(PC) --cflags libucw)
UCW_LDLIBS := $(shell $(PC) --libs libucw)

CFLAGS=$(UCW_CFLAGS) -O2 -std=gnu99 -Wall -Wextra -Wno-parentheses -Wstrict-prototypes -Wmissing-prototypes
LDLIBS=$(UCW_LDLIBS) -lz

all: dfu-sign

dfu-sign: dfu-sign.c

clean:
	rm -f dfu-sign *.o
