MODULE = phc-k8
VERSION = 0.4.6
# change KERNELSRC to the location of your kernel build tree only if
# autodetection does not work, e.g.:
#KERNELSRC=/usr/src/linux
# or run make with KERNELSRC parameter, e.g.:
# make KERNELSRC=/usr/src/linux
KERNELSRC ?= /lib/modules/`uname -r`/build
KERNELVERSION ?= $(shell grep '[\t ]UTS_RELEASE[\t ][\t ]*"' \
	$(KERNELSRC)/include/linux/version.h \
	$(KERNELSRC)/include/linux/utsrelease.h \
	$(KERNELSRC)/include/generated/utsrelease.h  2>/dev/null | \
	sed -e 's;[^"]*"\(.*\)";\1;g' )
DKMS_DEST ?= /usr/src/$(MODULE)-$(VERSION)

MODULES := phc-k8.ko 
obj-m += phc-k8.o

ifneq ($(BUILD_MPERF),)
	MODULES +=  mperf.ko
	obj-m += mperf.o
endif

ifeq ($(KERNELVERSION),)
$(error \
Kernel version not found, maybe you need to install appropriate kernel-headers \
or run make with KERNELSRC parameter, e.g.: make KERNELSRC=/usr/src/linux)
endif

all: $(MODULES)

help:
	@echo Possible targets:
	@echo -e all\\t\\t- \(default target\) build the kernel module
	@echo -e install\\t\\t- install the kernel module to /lib/modules/$(KERNELVERSION)/updates/kernel/arch/x86/kernel/cpu/cpufreq/
	@echo -e uninstall\\t- uninstall the kernel module
	@echo -e dkms_install\\t- install the module into the DKMS automated build system
	@echo -e dkms_uninstall\\t- uninstall the module from DKMS automated build system
	@echo -e clean\\t\\t- removes all binaries and temporary files

%.ko : %.c
	$(MAKE) -C $(KERNELSRC) SUBDIRS=$(PWD) $@

clean:
	rm -f *~ *.o *.s *.ko *.mod.c .*.cmd Module.symvers Module.markers modules.order
	rm -rf .tmp_versions

install: $(MODULES)
	install -m 644 -o root -g root phc-k8.modprobe /etc/modprobe.d/phc-k8.conf
	mkdir -p /lib/modules/$(KERNELVERSION)/updates/kernel/arch/x86/kernel/cpu/cpufreq/
	install -m 644 -o root -g root $(MODULES) /lib/modules/$(KERNELVERSION)/updates/kernel/arch/x86/kernel/cpu/cpufreq/
	depmod $(KERNELVERSION) -a

uninstall:
	rm /etc/modprobe.d/phc-k8.conf
	rm $(foreach target,$(MODULES),/lib/modules/$(KERNELVERSION)/updates/kernel/arch/x86/kernel/cpu/cpufreq/$(target))
	depmod $(KERNELVERSION) -a

dkms_check:
	@which dkms > /dev/null || ( echo \
	'**********************************************************************\n'\
	'Unable to locate "dkms". Did you forget to install it?\n'\
	'**********************************************************************\n' \
	&& exit 1)

dkms_mod_check:
	@if [ "$(shell dkms status -m $(MODULE))" ]; then \
	echo \
	'**********************************************************************\n'\
	"Please remove previous DKMS installs for $(MODULE) first:"; \
	dkms status -m $(MODULE); \
	echo \
	'For example: dkms remove -m $(MODULE) -v 0.4.0\n'\
	'**********************************************************************\n'\
	&& exit 1; \
	fi

dkms_install: dkms_check dkms_mod_check
	mkdir -p $(DKMS_DEST)
	install -m 644 -o root -g root Makefile dkms.conf phc-k8.c phc-k8.h mperf.c mperf.h $(DKMS_DEST)
	install -m 744 -o root -g root phc-k8.add $(DKMS_DEST)
	dkms add build install -m $(MODULE) -v $(VERSION)

dkms_uninstall: dkms_check
	dkms remove -m $(MODULE) -v $(VERSION) --all
