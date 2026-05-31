# define the module that is build
MODULE = cpu

# add a list of subdirectories, that should also be build
DIRS = periph $(RIOTCPU)/cortexm_common $(RIOTCPU)/nrf5x_common vectors

# build the nrf802154 driver if selected
ifneq (,$(filter nrf802154,$(USEMODULE)))
    DIRS += radio/nrf802154
endif

ifneq (,$(filter nrf_sf_radio_ppi,$(USEMODULE)))
    DIRS += nrf_sf_radio_ppi
endif

include $(RIOTBASE)/Makefile.base
