#-------------------------------------------------------------------
#
#-------------------------------------------------------------------

# - Try to find STLSOFT
# Once done, this will define
#
# STLSOFT_FOUND - system has STLSOFT
# STLSOFT_INCLUDE_DIRS - the STLSOFT include directories
# STLSOFT_LIBRARIES - link these to use STLSOFT

include(MyFindPkgMacros)
include(PrintListMacro)
findpkg_begin(STLSOFT)

# Get path, convert backslashes as ${ENV_${var}}
getenv_path(STLSOFT_HOME)

# construct search paths
set(STLSOFT_PREFIX_PATH ${STLSOFT_HOME} ${ENV_STLSOFT_HOME} /usr/local /usr/local/include /usr/local/lib /usr/include /usr/lib /usr/local/include/stlsoft /usr/include/stlsoft /usr/lib/stlsoft /usr/local/lib/stlsoft)

create_search_paths(STLSOFT)
#PRINTLIST("Search path:" "${STLSOFT_INC_SEARCH_PATH}")

# redo search if prefix path changed
clear_if_changed(STLSOFT_PREFIX_PATH
STLSOFT_LIBRARY_REL
STLSOFT_LIBRARY_DBG
STLSOFT_INCLUDE_DIR
)

set(STLSOFT_LIBRARY_NAMES "stlsoft")
get_release_debug_names(STLSOFT_LIBRARY_NAMES)

use_pkgconfig(STLSOFT_PKGC STLSOFT)

findpkg_framework(STLSOFT)

find_path(STLSOFT_INCLUDE_DIR NAMES "stlsoft/stlsoft.h" HINTS ${STLSOFT_INC_SEARCH_PATH} ${STLSOFT_PKGC_INCLUDE_DIRS} PATH_SUFFIXES stlsoft)
find_library(STLSOFT_LIBRARY_REL NAMES ${STLSOFT_LIBRARY_NAMES_REL} HINTS ${STLSOFT_LIB_SEARCH_PATH} ${STLSOFT_PKGC_LIBRARY_DIRS} PATH_SUFFIXES "" release relwithdebinfo minsizerel)
find_library(STLSOFT_LIBRARY_DBG NAMES ${STLSOFT_LIBRARY_NAMES_DBG} HINTS ${STLSOFT_LIB_SEARCH_PATH} ${STLSOFT_PKGC_LIBRARY_DIRS} PATH_SUFFIXES "" debug)

#Find the DLL's for the Runtime under Windows
IF(WIN32)
        set(STLSOFT_DLL_NAMES stlsoft)
        get_release_debug_filenames_dll(STLSOFT_DLL_NAMES)
        create_search_paths_dll(STLSOFT)
        FIND_FILE(STLSOFT_DLL_DBG ${STLSOFT_DLL_NAMES_REL} HINTS ${STLSOFT_DLL_SEARCH_PATH} ) 
        FIND_FILE(STLSOFT_DLL_REL ${STLSOFT_DLL_NAMES_DEL} HINTS ${STLSOFT_DLL_SEARCH_PATH} )
ENDIF(WIN32)


make_library_set(STLSOFT_LIBRARY)

findpkg_finish(STLSOFT TRUE)

add_parent_dir(STLSOFT_INCLUDE_DIRS STLSOFT_INCLUDE_DIR)

