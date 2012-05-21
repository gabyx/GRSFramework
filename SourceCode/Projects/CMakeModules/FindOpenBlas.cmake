#-------------------------------------------------------------------
#
#-------------------------------------------------------------------

# - Try to find OpenBlas
# Once done, this will define
#
# OPENBLAS_FOUND - system has OpenBlas
# OPENBLAS_INCLUDE_DIRS - the OpenBlas include directories
# OPENBLAS_LIBRARIES - link these to use OPENBLAS

include(MyFindPkgMacros)
include(PrintListMacro)
findpkg_begin(OPENBLAS)

# Get path, convert backslashes as ${ENV_${var}}
getenv_path(OPENBLAS_HOME)

# construct search paths
set(OPENBLAS_PREFIX_PATH ${OPENBLAS_HOME} ${ENV_OPENBLAS_HOME} /usr/local /usr/local/include /usr/local/lib /usr/include /usr/lib /usr/local/include/openblas /usr/include/openblas /usr/lib/openblasusr/local/lib/openblas)

create_search_paths(OPENBLAS)
#PRINTLIST("Search path:" "${OPENBLAS_INC_SEARCH_PATH}")

# redo search if prefix path changed
clear_if_changed(OPENBLAS_PREFIX_PATH
OPENBLAS_LIBRARY_REL
OPENBLAS_LIBRARY_DBG
OPENBLAS_INCLUDE_DIR
)

set(OPENBLAS_LIBRARY_NAMES "openblas")
get_release_debug_names(OPENBLAS_LIBRARY_NAMES)

use_pkgconfig(OPENBLAS_PKGC OPENBLAS)

findpkg_framework(OPENBLAS)

find_path(OPENBLAS_INCLUDE_DIR NAMES "cblas.h" HINTS ${OPENBLAS_INC_SEARCH_PATH} ${OPENBLAS_PKGC_INCLUDE_DIRS} PATH_SUFFIXES openblas)
find_library(OPENBLAS_LIBRARY_REL NAMES ${OPENBLAS_LIBRARY_NAMES_REL} HINTS ${OPENBLAS_LIB_SEARCH_PATH} ${OPENBLAS_PKGC_LIBRARY_DIRS} PATH_SUFFIXES "" release relwithdebinfo minsizerel)
find_library(OPENBLAS_LIBRARY_DBG NAMES ${OPENBLAS_LIBRARY_NAMES_DBG} HINTS ${OPENBLAS_LIB_SEARCH_PATH} ${OPENBLAS_PKGC_LIBRARY_DIRS} PATH_SUFFIXES "" debug)

#Find the DLL's for the Runtime under Windows
IF(WIN32)
        set(OPENBLAS_DLL_NAMES openblas)
        get_release_debug_filenames_dll(OPENBLAS_DLL_NAMES)
        create_search_paths_dll(OPENBLAS)
        FIND_FILE(OPENBLAS_DLL_DBG ${OPENBLAS_DLL_NAMES_REL} HINTS ${OPENBLAS_DLL_SEARCH_PATH} ) 
        FIND_FILE(OPENBLAS_DLL_REL ${OPENBLAS_DLL_NAMES_DEL} HINTS ${OPENBLAS_DLL_SEARCH_PATH} )
ENDIF(WIN32)


make_library_set(OPENBLAS_LIBRARY)

findpkg_finish(OPENBLAS)

add_parent_dir(OPENBLAS_INCLUDE_DIRS OPENBLAS_INCLUDE_DIR)

