!ifndef OUTPUT_DIR
OUTPUT_DIR=../bin
!endif

!ifndef CONFIG_DIR
CONFIG_DIR=../config
!endif

!ifndef INCLUDE_DIR
INCLUDE_DIR=../../include
!endif

!ifndef CONFIG_NAME
!error macro CONFIG_NAME not defined
!endif

!include "$(CONFIG_DIR)/$(CONFIG_NAME).conf"

brenchmark: log.txt

log.txt: benchmark.exe
	@echo -------------- Banchmark $(CONFIG_NAME) --------------
	#benchmark.exe >log.txt
	@echo ok

benchmark.exe: benchmark.cpp receivers.cpp
	$(CPP) $**
