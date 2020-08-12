
include eeprom/stdperiph_stm32f4/stm32lib.mk

EEPROMSRC =    eeprom/eeprom.c

EEPROMINC =    eeprom

ALLCSRC += $(EEPROMSRC)
ALLINC += $(EEPROMINC)
