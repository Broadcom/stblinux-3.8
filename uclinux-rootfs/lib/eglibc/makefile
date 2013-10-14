include $(ROOTDIR)/config.arch

unexport MAKEFILE_LIST

BOGUS_TARGETS		:= FORCE makefile $(ROOTDIR)/config.arch

# This guard is necessary because the generated Makefile in the build folder
# does make -C on this directory causing an infinite loop.
ifneq ($(EGLIBC_MAKEFILE_GUARD),1)
EGLIBC_MAKEFILE_GUARD = 1

.PHONY: all
all::
	if [ ! -e build/Makefile ]; then \
		mkdir -p build; \
		cd build; \
		../configure --host=$(shell $(CC) -dumpmachine) \
			--prefix= \
			--with-sysroot=$(shell $(CC) -print-sysroot) \
			--enable-obsolete-rpc \
			--disable-nscd; \
	fi
	$(MAKE) -C build

.PHONY: distclean
distclean::
	rm -rf build temp

# We install to a temporary folder first and generate the list of
# files we actually want from that folder, then copy those to the romfs.
# This is based off of Fedora's glibc RPM spec.
.PHONY: romfs
romfs::
	mkdir -p temp
	make -C build install DESTDIR=$(CURDIR)/temp
	mkdir -p $(ROMFSDIR)/lib
	cp -df temp/lib/*.so* $(ROMFSDIR)/lib
	$(ROMFSINST) -e CONFIG_LIB_EGLIBC_LDD temp/bin/ldd /bin/ldd
	$(ROMFSINST) -e CONFIG_LIB_EGLIBC_LDCONFIG temp/sbin/ldconfig /sbin/ldconfig

endif # EGLIBC_MAKEFILE_GUARD

# These targets must not be passed through to the original Makefile
.PHONY: $(BOGUS_TARGETS)
$(BOGUS_TARGETS)::

# Everything else (maybe including clean, distclean) does get passed through
.NOTPARALLEL: %
%:: FORCE
	$(MAKE) -f Makefile "$@"
