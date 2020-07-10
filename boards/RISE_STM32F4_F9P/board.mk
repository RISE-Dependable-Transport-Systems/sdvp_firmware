# List of all the board related files.
BOARDSRC = ./boards/RISE_STM32F4_F9P/board.c

# Required include directories
BOARDINC = ./boards/RISE_STM32F4_F9P

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
