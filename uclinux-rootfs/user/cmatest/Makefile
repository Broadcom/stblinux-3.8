include $(ROOTDIR)/config.arch

LINUXDIR ?= linux

CFLAGS += -I. -I$(ROOTDIR)/$(LINUXDIR)/include/uapi -Wall -Wextra -Werror -D_GNU_SOURCE

.PHONY: all
all: cmatest

cmatest: cmatest.c
	$(CC) $(CFLAGS) $< -o $@

.PHONY: clean distclean romfs
clean:
	rm -f cmatest

distclean: clean

romfs:
	$(ROMFSINST) -e CONFIG_USER_CMATEST cmatest /bin/cmatest
