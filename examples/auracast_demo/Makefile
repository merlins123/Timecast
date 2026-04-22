APPLICATION = auracast_demo

BOARD ?= feather-nrf52840-sense

RIOTBASE ?= $(CURDIR)/../..

USEPKG += nimble

USEMODULE += nimble_autoadv
USEMODULE += nimble_svc_gap
USEMODULE += ztimer_msec

CFLAGS += -DCONFIG_NIMBLE_AUTOADV_DEVICE_NAME='"RIOT Auracast Demo"'
CFLAGS += -DCONFIG_NIMBLE_AUTOADV_START_MANUALLY=1

DEVELHELP ?= 1
QUIET ?= 1

include $(RIOTBASE)/Makefile.include
