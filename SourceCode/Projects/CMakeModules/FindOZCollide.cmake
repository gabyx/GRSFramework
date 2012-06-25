#-------------------------------------------------------------------
#
#-------------------------------------------------------------------

# - Try to find OzCollide
# Once done, this will define
#
# OZCOLLIDE_FOUND - system has OzCollide
# OZCOLLIDE_INCLUDE_DIRS - the OzCollide include directories
# OZCOLLIDE_LIBRARIES - link these to use OzCollide

include(MyFindPkgMacros)
include(PrintListMacro)
findpkg_begin(OZCOLLIDE)

# Get path, convert backslashes as ${ENV_${var}}
getenv_path(OZCOLLIDE_HOME)

# construct search paths
set(OZCOLLIDE_PREFIX_PATH ${OZCOLLIDE_HOME} ${ENV_OZCOLLIDE_HOME} /usr/local /usr/local/include /usr/local/lib /usr/include /usr/lib /usr/local/include/ozcollide /usr/include/ozcollide /usr/lib/ozcollide /usr/local/lib/ozcollide)

create_search_paths(OZCOLLIDE)
#PRINTLIST("Search path:" "${OZCOLLIDE_INC_SEARCH_PATH}")

# redo search if prefix path changed
clear_if_changed(OZCOLLIDE_PREFIX_PATH
OZCOLLIDE_LIBRARY_REL
OZCOLLIDE_LIBRARY_DBG
OZCOLLIDE_INCLUDE_DIR
)

set(OZCOLLIDE_LIBRARY_NAMES "ozcollide")
get_release_debug_names(OZCOLLIDE_LIBRARY_NAMES)

use_pkgconfig(OZCOLLIDE_PKGC OZCOLLIDE)

findpkg_framework(OZCOLLIDE)

find_path(OZCOLLIDE_INCLUDE_DIR NAMES "ozcollide.h" HINTS ${OZCOLLIDE_INC_SEARCH_PATH} ${OZCOLLIDE_PKGC_INCLUDE_DIRS} PATH_SUFFIXES ozcollide)
find_library(OZCOLLIDE_LIBRARY_REL NAMES ${OZCOLLIDE_LIBRARY_NAMES_REL} HINTS ${OZCOLLIDE_LIB_SEARCH_PATH} ${OZCOLLIDE_PKGC_LIBRARY_DIRS} PATH_SUFFIXES "" release relwithdebinfo minsizerel)
find_library(OZCOLLIDE_LIBRARY_DBG NAMES ${OZCOLLIDE_LIBRARY_NAMES_DBG} HINTS ${OZCOLLIDE_LIB_SEARCH_PATH} ${OZCOLLIDE_PKGC_LIBRARY_DIRS} PATH_SUFFIXES "" debug)
make_library_set(OZCOLLIDE_LIBRARY)


#Find the DLL's for the Runtime under Windows
IF(WIN32)
        set(OZCOLLIDE_DLL_NAMES ozcollide)
        get_release_debug_filenames_dll(OZCOLLIDE_DLL_NAMES)
        create_search_paths_dll(OZCOLLIDE)
        FIND_FILE(OZCOLLIDE_DLL_DBG ${OZCOLLIDE_DLL_NAMES_REL} HINTS ${OZCOLLIDE_DLL_SEARCH_PATH} ) 
        FIND_FILE(OZCOLLIDE_DLL_REL ${OZCOLLIDE_DLL_NAMES_DEL} HINTS ${OZCOLLIDE_DLL_SEARCH_PATH} )
ENDIF(WIN32)


make_library_set(OZCOLLIDE_LIBRARY)

findpkg_finish(OZCOLLIDE)

add_parent_dir(OZCOLLIDE_INCLUDE_DIRS OZCOLLIDE_INCLUDE_DIR)

