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

!ifdef PREFERRED_SYNTAX_ALLOWED
runall: compatibility_syntax_tests preferred_syntax_tests events
!else
runall: compatibility_syntax_tests events
!endif

compatibility_syntax_tests: IncludeTest.exe InstanceTest.exe MakeTest.exe
	MakeTest.exe

preferred_syntax_tests: IncludeTestP.exe InstanceTestP.exe

#events: BoostFunctionEventTest.exe DelegateEventTest.exe InterfaceEventTest.exe
events: DelegateEventTest.exe InterfaceEventTest.exe

IncludeTest.exe: IncludeTest.cpp
IncludeTestP.exe: IncludeTestP.cpp
InstanceTest.exe: InstanceTest.cpp
InstanceTestP.exe: InstanceTestP.cpp
MakeTest.exe: MakeTest.cpp
BoostFunctionEventTest.exe: BoostFunctionEventTest.cpp
DelegateEventTest.exe: DelegateEventTest.cpp
InterfaceEventTest.exe: InterfaceEventTest.cpp
