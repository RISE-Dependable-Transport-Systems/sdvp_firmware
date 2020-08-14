
include $(COMMONDIR)/eeprom/stdperiph_stm32f4/stm32lib.mk

EEPROMSRC =    $(COMMONDIR)/eeprom/eeprom.c

EEPROMINC =    $(COMMONDIR)/eeprom

ALLCSRC += $(EEPROMSRC)
ALLINC += $(EEPROMINC)
