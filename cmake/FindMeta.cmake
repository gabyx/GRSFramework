# Find the Meta include directory
# The following variables are set if Meta is found.
#  Meta_FOUND        - True when the Meta include directory is found.
#  Meta_INCLUDE_DIR  - The path to where the meta include files are.
# If Meta is not found, Meta_FOUND is set to false.

find_package(PkgConfig)


if(NOT EXISTS "${Meta_INCLUDE_DIR}")
  find_path(Meta_INCLUDE_DIR
    NAMES meta/meta.hpp 
    DOC "Meta library header files"
    )
endif()

if(EXISTS "${Meta_INCLUDE_DIR}")

else()
  message(STATUS "Meta: Setup External Projext")
  include(ExternalProject)
  ExternalProject_Add(meta
    GIT_REPOSITORY https://github.com/ericniebler/meta.git
    TIMEOUT 10
    CMAKE_ARGS -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER} -DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}
    PREFIX "${CMAKE_CURRENT_BINARY_DIR}"
    INSTALL_COMMAND "" # Disable install step
    )
  
  # Specify include dir
  ExternalProject_Get_Property(meta source_dir)
  set(Meta_INCLUDE_DIR ${source_dir}/include)
endif()


include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Meta DEFAULT_MSG Meta_INCLUDE_DIR)
mark_as_advanced(Meta_INCLUDE_DIR)