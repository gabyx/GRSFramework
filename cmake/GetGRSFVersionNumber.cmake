# define version number of the GRSFramework 
# GRSF_VERSION, 
#  GRSF_VERSION_MAJOR,
#  GRSF_VERSION_MINOR, 
#  GRSF_VERSION_PATCH, 
#  GRSF_VERSION_SHA1, 
#  GRSF_VERSION_STRING


macro(GetGRSFVersionNumber)

# Get GRSFramework version number ======================================
    
    include(GetGitRevisionDescription)
    git_describe(GRSF_VERSION "--tags" "--abbrev=10")

    if( NOT GRSF_VERSION )
    message(FATAL_ERROR "GRSFramework version could not be determined!, ${GRSF_VERSION}")
    endif()

    string(REGEX REPLACE "^.*v([0-9]+)\\..*" "\\1" GRSF_VERSION_MAJOR "${GRSF_VERSION}")
    string(REGEX REPLACE "^.*v[0-9]+\\.([0-9]+).*" "\\1" GRSF_VERSION_MINOR "${GRSF_VERSION}")
    string(REGEX REPLACE "^.*v[0-9]+\\.[0-9]+\\.([0-9]+).*" "\\1" GRSF_VERSION_PATCH "${GRSF_VERSION}")
    string(REGEX REPLACE "^.*v[0-9]+\\.[0-9]+\\.[0-9]+-[0-9]*-([0-9a-z]*)" "\\1" GRSF_VERSION_SHA1 "${GRSF_VERSION}")

    if(NOT ${GRSF_VERSION_SHA1} STREQUAL "")
        set(GRSF_VERSION_STRING "${GRSF_VERSION_MAJOR}.${GRSF_VERSION_MINOR}.${GRSF_VERSION_PATCH}-${GRSF_VERSION_SHA1}")
    else()
        set(GRSF_VERSION_STRING "${GRSF_VERSION_MAJOR}.${GRSF_VERSION_MINOR}.${GRSF_VERSION_PATCH}")
    endif()
    MESSAGE(STATUS "GRSFramework Version: ${GRSF_VERSION_STRING} extracted from git tags!")

# ======================================================================


endmacro()
