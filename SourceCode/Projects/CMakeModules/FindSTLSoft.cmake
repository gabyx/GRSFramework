# Find STLSOFT Template Library
#
# STLSOFT_FOUND
# STLSOFT_INCLUDE_DIR      where to find the include files


if("$ENV{STLSOFT}")
message(FATAL_ERROR "Environment STLSOFT variable not set!")
endif()

if(NOT $ENV{STLSOFT})

endif(NOT $ENV{STLSOFT})

message(STATUS "Environment STLSOFT variable set to: $ENV{STLSOFT}")
FIND_PATH(STLSOFT_INCLUDE_DIR "platformstl/platformstl.h" PATHS "$ENV{STLSOFT}/include") 
message(STATUS "STLSoft Include Dir: " ${STLSOFT_INCLUDE_DIR})
STRING(COMPARE NOTEQUAL ${STLSOFT_INCLUDE_DIR}  "STLSOFT_INCLUDE_DIR-NOTFOUND" STLSOFT_FOUND)

IF(NOT STLSOFT_FOUND)
	IF(STLSOFT_FIND_REQUIRED)
		message( FATAL_ERROR "STLSoft Include directory was not found!")
	ELSE(STLSOFT_FIND_REQUIRED)
		message( STATUS "STLSoft Include directory was not found!")
	ENDIF(STLSOFT_FIND_REQUIRED)
ELSE(NOT STLSOFT_FOUND)
	message(STATUS "STLSoft Include directory found!")
ENDIF(NOT STLSOFT_FOUND)


MARK_AS_ADVANCED( STLSOFT_INCLUDE_DIR )
