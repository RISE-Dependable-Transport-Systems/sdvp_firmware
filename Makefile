##############################################################################
# Multi-project makefile rules
#

.PHONY: rover copter

all: rover copter

rover:
	@echo
	@echo === Building rover firmware ========================================
	+@make --no-print-directory -f ./rover/rover.make all
	@echo ====================================================================
	@echo

copter:
	@echo
	@echo === Building copter firmware =======================================
	+@make --no-print-directory -f ./copter/copter.make all
	@echo ====================================================================
	@echo

clean:
	@echo
	+@make --no-print-directory -f ./rover/rover.make clean
	@echo
	+@make --no-print-directory -f ./copter/copter.make clean
	@echo

#
############################################################################## 
