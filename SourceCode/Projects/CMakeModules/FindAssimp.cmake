#-------------------------------------------------------------------
#
#-------------------------------------------------------------------

# - Try to find Assimp
# Once done, this will define
#
# ASSIMP_FOUND - system has Assimp
# ASSIMP_INCLUDE_DIR - the Assimp include directories
# ASSIMP_LIBRARY - link these to use Assimp

include(MyFindPkgMacros)
include(PrintListMacro)
findpkg_begin(ASSIMP)

if(NOT Assimp_FIND_VERSION)
  if(NOT Assimp_FIND_VERSION_MAJOR)
    set(Assimp_FIND_VERSION_MAJOR 3)
  endif()
  if(NOT Assimp_FIND_VERSION_MINOR)
    set(Assimp_FIND_VERSION_MINOR 0)
  endif()
  if(NOT Assimp_FIND_VERSION_PATCH)
    set(Assimp_FIND_VERSION_PATCH 0)
  endif()
  set(Assimp_FIND_VERSION "${Assimp_FIND_VERSION_MAJOR}.${Assimp_FIND_VERSION_MINOR}.${Assimp_FIND_VERSION_PATCH}")
endif()

if(Assimp_FIND_VERSION_MAJOR VERSION_LESS 3)
set(ASSIMP_SEARCH_INCLUDE_FILES "assimp.hpp")
else()
set(ASSIMP_SEARCH_INCLUDE_FILES "Importer.hpp")
endif()

# Get path, convert backslashes as ${ENV_${var}}
getenv_path(ASSIMP_HOME)

# construct search paths
set(ASSIMP_PREFIX_PATH ${ASSIMP_HOME} ${ENV_ASSIMP_HOME} /usr/local /usr/local/include /usr/local/lib /usr/include /usr/lib /usr/local/include/assimp /usr/include/assimp /usr/lib/assimp /usr/local/lib/assimp)

create_search_paths(ASSIMP)
PRINTLIST("Search path:" "${ASSIMP_INC_SEARCH_PATH}")

# redo search if prefix path changed
clear_if_changed(ASSIMP_PREFIX_PATH
ASSIMP_LIBRARY_REL
ASSIMP_LIBRARY_DBG
ASSIMP_INCLUDE_DIR
)

set(ASSIMP_LIBRARY_NAMES assimp)
get_release_debug_names(ASSIMP_LIBRARY_NAMES)

use_pkgconfig(ASSIMP_PKGC ASSIMP)

findpkg_framework(ASSIMP)

find_path(ASSIMP_INCLUDE_DIR NAMES ${ASSIMP_SEARCH_INCLUDE_FILES} HINTS ${ASSIMP_INC_SEARCH_PATH} ${ASSIMP_PKGC_INCLUDE_DIRS} PATH_SUFFIXES assimp)
find_library(ASSIMP_LIBRARY_REL NAMES ${ASSIMP_LIBRARY_NAMES_REL} HINTS ${ASSIMP_LIB_SEARCH_PATH} ${ASSIMP_PKGC_LIBRARY_DIRS} PATH_SUFFIXES "" release relwithdebinfo minsizerel)
find_library(ASSIMP_LIBRARY_DBG NAMES ${ASSIMP_LIBRARY_NAMES_DBG} HINTS ${ASSIMP_LIB_SEARCH_PATH} ${ASSIMP_PKGC_LIBRARY_DIRS} PATH_SUFFIXES "" debug)

#Find the DLL's for the Runtime under Windows
IF(WIN32)
        set(ASSIMP_DLL_NAMES assimp)
        get_release_debug_filenames_dll(ASSIMP_DLL_NAMES)
        create_search_paths_dll(ASSIMP)
        FIND_FILE(ASSIMP_DLL_DBG ${ASSIMP_DLL_NAMES_REL} HINTS ${ASSIMP_DLL_SEARCH_PATH} ) 
        FIND_FILE(ASSIMP_DLL_REL ${ASSIMP_DLL_NAMES_DEL} HINTS ${ASSIMP_DLL_SEARCH_PATH} )
ENDIF(WIN32)


make_library_set(ASSIMP_LIBRARY)

findpkg_finish(ASSIMP)

add_parent_dir(ASSIMP_INCLUDE_DIRS ASSIMP_INCLUDE_DIR)

include(FindPackageHandleStandardArgs)
## handle the QUIETLY and REQUIRED arguments and set LIBXML2_FOUND to TRUE
## if all listed variables are TRUE
find_package_handle_standard_args(ASSIMP DEFAULT_MSG ASSIMP_LIBRARY ASSIMP_INCLUDE_DIR)


