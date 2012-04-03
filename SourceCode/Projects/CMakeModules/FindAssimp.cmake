# Find Collision Library Opcode
#
# ASSIMP_FOUND
# ASSIMP_INCLUDE_DIR      where to find the include files
# ASSIMP_LIB_REL
# ASSIMP_LIB_DBG
# ASSIMP_REL
# ASSIMP_DBG
# ASSIMP_LIBRARIES


SET(ASSIMP_SEARCH "C:/Develop/Assimp" "C:/Develop/ASSIMP")


FIND_PATH(ASSIMP_INCLUDE_DIR "assimp.hpp" HINTS ${ASSIMP_SEARCH} PATH_SUFFIXES "include" "include/assimp" ) 

FIND_LIBRARY(ASSIMP_LIB_REL "assimp.lib" HINTS ${ASSIMP_SEARCH} PATH_SUFFIXES "lib/Release" "lib/release" ) 
FIND_LIBRARY(ASSIMP_LIB_DBG "assimp.lib" HINTS ${ASSIMP_SEARCH} PATH_SUFFIXES "lib/Debug" "lib/debug" ) 
FIND_FILE(ASSIMP_REL "assimp.dll" HINTS ${ASSIMP_SEARCH} PATH_SUFFIXES "bin/Debug" "bin/debug"  "lib/Debug" "lib/debug" ) 
FIND_FILE(ASSIMP_DBG "assimp.dll" HINTS ${ASSIMP_SEARCH} PATH_SUFFIXES "bin/Debug" "bin/debug"  "lib/Debug" "lib/debug" ) 

SET(ASSIMP_LIBRARIES "optimized" ${ASSIMP_LIB_REL} "debug" ${ASSIMP_LIB_DBG})

message(STATUS "ASSIMP Include Dir: " ${ASSIMP_INCLUDE_DIR})
STRING(COMPARE NOTEQUAL ${ASSIMP_INCLUDE_DIR}  "ASSIMP_INCLUDE_DIR-NOTFOUND" ASSIMP_FOUND)

IF(NOT ASSIMP_FOUND)
	IF(ASSIMP_FIND_REQUIRED)
		message( FATAL_ERROR "ASSIMP  Include directory was not found!")
	ELSE(ASSIMP_FIND_REQUIRED)
		message( STATUS "ASSIMP  Include directory was not found!")
	ENDIF(ASSIMP_FIND_REQUIRED)
ELSE(NOT ASSIMP_FOUND)
	message(STATUS "ASSIMP  Include directory found!")
ENDIF(NOT ASSIMP_FOUND)

MARK_AS_ADVANCED( ASSIMP_INCLUDE_DIR )

