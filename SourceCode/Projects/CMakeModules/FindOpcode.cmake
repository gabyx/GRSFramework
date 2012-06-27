#-------------------------------------------------------------------
#
#-------------------------------------------------------------------

# - Try to find OPCODE
# Once done, this will define
#
# OPCODE_FOUND - system has OPCODE
# OPCODE_INCLUDE_DIRS - the OPCODE include directories
# OPCODE_LIBRARIES - link these to use OPCODE

include(MyFindPkgMacros)
include(PrintListMacro)
findpkg_begin(OPCODE)

# Get path, convert backslashes as ${ENV_${var}}
getenv_path(OPCODE_HOME)

# construct search paths
set(OPCODE_PREFIX_PATH ${OPCODE_HOME} ${ENV_OPCODE_HOME} /usr/local /usr/local/include /usr/local/lib /usr/include /usr/lib /usr/local/include/opcode /usr/include/opcode /usr/lib/opcode /usr/local/lib/opcode)

create_search_paths(OPCODE)
PRINTLIST("Search path:" "${OPCODE_INC_SEARCH_PATH}")

# redo search if prefix path changed
clear_if_changed(OPCODE_PREFIX_PATH
OPCODE_LIBRARY_REL
OPCODE_LIBRARY_DBG
OPCODE_INCLUDE_DIR
)

set(OPCODE_LIBRARY_NAMES "opcode")
get_release_debug_names(OPCODE_LIBRARY_NAMES)

use_pkgconfig(OPCODE_PKGC OPCODE)

findpkg_framework(OPCODE)

SET(OPCODE_HEADER_NAMES "Opcode.h" "opcode.h")

find_path(OPCODE_INCLUDE_DIR NAMES ${OPCODE_HEADER_NAMES} HINTS ${OPCODE_INC_SEARCH_PATH} ${OPCODE_PKGC_INCLUDE_DIRS} PATH_SUFFIXES opcode)
find_library(OPCODE_LIBRARY_REL NAMES ${OPCODE_LIBRARY_NAMES_REL} HINTS ${OPCODE_LIB_SEARCH_PATH} ${OPCODE_PKGC_LIBRARY_DIRS} PATH_SUFFIXES "" release relwithdebinfo minsizerel)
find_library(OPCODE_LIBRARY_DBG NAMES ${OPCODE_LIBRARY_NAMES_DBG} HINTS ${OPCODE_LIB_SEARCH_PATH} ${OPCODE_PKGC_LIBRARY_DIRS} PATH_SUFFIXES "" debug)
make_library_set(OPCODE_LIBRARY)


#Find the DLL's for the Runtime under Windows
IF(WIN32)
        set(OPCODE_DLL_NAMES OPCODE)
        get_release_debug_filenames_dll(OPCODE_DLL_NAMES)
        create_search_paths_dll(OPCODE)
        FIND_FILE(OPCODE_DLL_DBG ${OPCODE_DLL_NAMES_REL} HINTS ${OPCODE_DLL_SEARCH_PATH} ) 
        FIND_FILE(OPCODE_DLL_REL ${OPCODE_DLL_NAMES_DEL} HINTS ${OPCODE_DLL_SEARCH_PATH} )
ENDIF(WIN32)


make_library_set(OPCODE_LIBRARY)

findpkg_finish(OPCODE)

add_parent_dir(OPCODE_INCLUDE_DIRS OPCODE_INCLUDE_DIR)

