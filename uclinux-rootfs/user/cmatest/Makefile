include $(ROOTDIR)/config.arch

CFLAGS += -I. -I../../../linux/include/uapi -Wall -Wextra -Werror

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
