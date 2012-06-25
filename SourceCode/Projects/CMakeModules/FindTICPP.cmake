#-------------------------------------------------------------------
#
#-------------------------------------------------------------------

# - Try to find TinyXML++
# Once done, this will define
#
# TICPP_FOUND - system has TinyXML++
# TICPP_INCLUDE_DIRS - the TinyXML++ include directories
# TICPP_LIBRARIES - link these to use TinyXML++

include(MyFindPkgMacros)
include(PrintListMacro)
findpkg_begin(TICPP)

# Get path, convert backslashes as ${ENV_${var}}
getenv_path(TICPP_HOME)

# construct search paths
set(TICPP_PREFIX_PATH ${TICPP_HOME} ${ENV_TICPP_HOME} /usr/local /usr/local/include /usr/local/lib /usr/include /usr/lib /usr/local/include/ticpp /usr/include/ticpp /usr/lib/ticpp /usr/local/lib/ticpp)

create_search_paths(TICPP)
PRINTLIST("Search path:" "${TICPP_INC_SEARCH_PATH}")

# redo search if prefix path changed
clear_if_changed(TICPP_PREFIX_PATH
TICPP_LIBRARY_REL
TICPP_LIBRARY_DBG
TICPP_INCLUDE_DIR
)

set(TICPP_LIBRARY_NAMES "ticpp")
get_release_debug_names(TICPP_LIBRARY_NAMES)

use_pkgconfig(TICPP_PKGC TICPP)

findpkg_framework(TICPP)

find_path(TICPP_INCLUDE_DIR NAMES "ticpp.h" HINTS ${TICPP_INC_SEARCH_PATH} ${TICPP_PKGC_INCLUDE_DIRS} PATH_SUFFIXES ticpp)
find_library(TICPP_LIBRARY_REL NAMES ${TICPP_LIBRARY_NAMES_REL} HINTS ${TICPP_LIB_SEARCH_PATH} ${TICPP_PKGC_LIBRARY_DIRS} PATH_SUFFIXES "" release relwithdebinfo minsizerel)
find_library(TICPP_LIBRARY_DBG NAMES ${TICPP_LIBRARY_NAMES_DBG} HINTS ${TICPP_LIB_SEARCH_PATH} ${TICPP_PKGC_LIBRARY_DIRS} PATH_SUFFIXES "" debug)
make_library_set(TICPP_LIBRARY)

#Find the DLL's for the Runtime under Windows
IF(WIN32)
        set(TICPP_DLL_NAMES ticpp)
        get_release_debug_filenames_dll(TICPP_DLL_NAMES)
        create_search_paths_dll(TICPP)
        FIND_FILE(TICPP_DLL_DBG ${TICPP_DLL_NAMES_REL} HINTS ${TICPP_DLL_SEARCH_PATH} ) 
        FIND_FILE(TICPP_DLL_REL ${TICPP_DLL_NAMES_DEL} HINTS ${TICPP_DLL_SEARCH_PATH} )
ENDIF(WIN32)


findpkg_finish(TICPP)


add_parent_dir(TICPP_INCLUDE_DIRS TICPP_INCLUDE_DIR)

