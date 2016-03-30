# this file is executed outside of ApproxMVBB to get the revision describtion
include(cmake/GetGitRevisionDescription.cmake)
git_describe(GRSF_VERSION "--tags" "--abbrev=0")

if( NOT GRSF_VERSION )
message(FATAL_ERROR "GRSFramwork version could not be determined!, ${GRSF_VERSION}")
endif()

message(STATUS "${GRSF_VERSION}")