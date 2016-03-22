#-------------------------------------------------------------------
#
#-------------------------------------------------------------------

# - Try to find gccfilter  http://www.mixtion.org/gccfilter/
# Once done, this will define
#
# GCCFILTER_FOUND - system has GCCFILTER
# GCCFILTER_BIN - the gccfilter execute directory
# GCCFILTER_ADDITIONAL_FLAGS - additional flags which are passed to the gccfilter

include(MyFindPkgMacros)
include(PrintListMacro)
findpkg_begin(GCCFILTER)

# Get path, convert backslashes as ${ENV_${var}}
getenv_path(GCCFILTER_HOME)

# construct search paths
set(GCCFILTER_PREFIX_PATH ${GCCFILTER_HOME} ${ENV_GCCFILTER_HOME} /usr/local /usr/local/include /usr/local/lib /usr/include /usr/lib /usr/local/include/GCCFILTER /usr/include/GCCFILTER /usr/lib/GCCFILTER /usr/local/lib/GCCFILTER)

create_search_paths(GCCFILTER)
PRINTLIST("Search path:" "${GCCFILTER_INC_SEARCH_PATH}")

# redo search if prefix path changed
clear_if_changed(GCCFILTER_PREFIX_PATH
	GCCFILTER_BIN
)


find_file(GCCFILTER_BIN NAMES gccfilter HINTS ${GCCFILTER_INC_SEARCH_PATH} PATH_SUFFIXES gccfilter)




include(FindPackageHandleStandardArgs)
## handle the QUIETLY and REQUIRED arguments and set LIBXML2_FOUND to TRUE
## if all listed variables are TRUE
find_package_handle_standard_args(GCCFILTER DEFAULT_MSG GCCFILTER_BIN)

if( GCCFILTER_FOUND)
	set(GCCFILTER_ADDITIONAL_FLAGS "--colorize --remove-template-args" CACHE STRING "additional flags to the gccfilter tool")
endif()
