#-------------------------------------------------------------------
#
#-------------------------------------------------------------------

# - Try to find PNetCDF
# Once done, this will define
#
# PNETCDF_FOUND - system has PNetCDF
# PNETCDF_INCLUDE_DIR - the PNetCDF include directories
# PNETCDF_LIBRARY - link these to use PNetCDF

include(MyFindPkgMacros)
include(PrintListMacro)
findpkg_begin(PNETCDF)

# Get path, convert backslashes as ${ENV_${var}}
getenv_path(PNETCDF_HOME)

# construct search paths
set(PNETCDF_PREFIX_PATH ${PNETCDF_HOME} ${ENV_PNETCDF_HOME} /usr/local /usr/local/include /usr/local/lib /usr/include /usr/lib /usr/local/include/pnetcdf /usr/include/pnetcdf /usr/lib/pnetcdf /usr/local/lib/pnetcdf)

create_search_paths(PNETCDF)
PRINTLIST("Search path:" "${PNETCDF_INC_SEARCH_PATH}")

# redo search if prefix path changed
clear_if_changed(PNETCDF_PREFIX_PATH
PNETCDF_LIBRARY_REL
PNETCDF_LIBRARY_DBG
PNETCDF_INCLUDE_DIR
)

set(PNETCDF_LIBRARY_NAMES "pnetcdf")
get_release_debug_names(PNETCDF_LIBRARY_NAMES)

use_pkgconfig(PNETCDF_PKGC PNETCDF)
PRINTLIST("Search path PKGConfig:" "${PNETCDF_PKGC_INCLUDE_DIRS}")

findpkg_framework(PNETCDF)

find_path(PNETCDF_INCLUDE_DIR NAMES "pnetcdf.h" HINTS ${PNETCDF_INC_SEARCH_PATH} ${PNETCDF_PKGC_INCLUDE_DIRS} PATH_SUFFIXES pnetcdf)
find_library(PNETCDF_LIBRARY_REL NAMES ${PNETCDF_LIBRARY_NAMES_REL} HINTS ${PNETCDF_LIB_SEARCH_PATH} ${PNETCDF_PKGC_LIBRARY_DIRS} PATH_SUFFIXES "" release relwithdebinfo minsizerel)
find_library(PNETCDF_LIBRARY_DBG NAMES ${PNETCDF_LIBRARY_NAMES_DBG} HINTS ${PNETCDF_LIB_SEARCH_PATH} ${PNETCDF_PKGC_LIBRARY_DIRS} PATH_SUFFIXES "" debug)
make_library_set(PNETCDF_LIBRARY)

#Find the DLL's for the Runtime under Windows
IF(WIN32)
        set(PNETCDF_DLL_NAMES pnetcdf)
        get_release_debug_filenames_dll(PNETCDF_DLL_NAMES)
        create_search_paths_dll(PNETCDF)
        FIND_FILE(PNETCDF_DLL_DBG ${PNETCDF_DLL_NAMES_REL} HINTS ${PNETCDF_DLL_SEARCH_PATH} ) 
        FIND_FILE(PNETCDF_DLL_REL ${PNETCDF_DLL_NAMES_DEL} HINTS ${PNETCDF_DLL_SEARCH_PATH} )
ENDIF(WIN32)

findpkg_finish(PNETCDF)

add_parent_dir(PNETCDF_INCLUDE_DIRS PNETCDF_INCLUDE_DIR)


include(FindPackageHandleStandardArgs)
## handle the QUIETLY and REQUIRED arguments and set LIBXML2_FOUND to TRUE
## if all listed variables are TRUE
find_package_handle_standard_args(Ticpp DEFAULT_MSG PNETCDF_LIBRARY PNETCDF_INCLUDE_DIR)

