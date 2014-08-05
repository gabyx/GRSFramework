#The name of the project
PROJECT(PythonCWrapper)

#The CMake Minimum version that is required. The FindCUDA script
#is distributed since version 2.8
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

SET(MYPROJECT_LIBRARY_PATH "" CACHE STRING "Where cmake searches for libraries (on cluster change this value)")
SET(CMAKE_LIBRARY_PATH ${MYPROJECT_LIBRARY_PATH} ${CMAKE_LIBRARY_PATH} )


set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../CMakeModules/;${CMAKE_MODULE_PATH}")
message(STATUS "Module path is now set to: " ${CMAKE_MODULE_PATH} )

# Set the makefile build type (only for make file based generators)
MARK_AS_ADVANCED( CMAKE_BUILD_TYPE )
set(CMAKE_BUILD_TYPE Debug CACHE STRING "The build type for makefile based generators")


# If GNU C Compiler set warning lenght to one line for eclipse
MESSAGE(STATUS "Compiler ID is: " ${CMAKE_CXX_COMPILER_ID})
IF(${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")

	MESSAGE(STATUS "Setting Values for GCC")
	SET(CMAKE_C_FLAGS "-fmessage-length=0" CACHE STRING "Flags for C Compiler" FORCE)
	SET(CMAKE_CXX_FLAGS "-std=c++11 -ftree-vectorize -ftree-vectorizer-verbose=0 -fmessage-length=0 -Wno-enum-compare" CACHE STRING "Flags for CXX Compiler" FORCE)
	
ELSEIF ( ${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang" )

	MESSAGE(STATUS "Setting Values for CLANG")
    SET (CMAKE_C_FLAGS                "-Wall -std=c99" CACHE STRING "Flags for C Compiler" FORCE)
    SET (CMAKE_C_FLAGS_DEBUG          "-g" CACHE STRING "Flags for C Compiler for debug builds" FORCE)
    SET (CMAKE_C_FLAGS_MINSIZEREL     "-Os -DNDEBUG" CACHE STRING "Flags for C Compiler for release minsize builds" FORCE)
    SET (CMAKE_C_FLAGS_RELEASE        "-O4 -DNDEBUG" CACHE STRING "Flags for C Compiler for release builds" FORCE)
    SET (CMAKE_C_FLAGS_RELWITHDEBINFO "-O2 -g" CACHE STRING "Flags for C Compiler for release builds with debug info" FORCE)
    
    #SET (CMAKE_CXX_FLAGS                "-Weverything -Wno-covered-switch -Wno-documentation -pedantic -Wno-conversion -Wno-comment -Wno-unused-parameter -Wno-deprecated-register -Wno-float-equal -Wno-switch -Wextra-semi -fdiagnostics-show-template-tree")
    SET (CMAKE_CXX_FLAGS                "-w" CACHE STRING "Flags for CXX Compiler" FORCE)
    SET (CMAKE_CXX_FLAGS_DEBUG          "-g" CACHE STRING "Flags for CXX Compiler for debug builds" FORCE)
    SET (CMAKE_CXX_FLAGS_MINSIZEREL     "-Os -DNDEBUG" CACHE STRING "Flags for CXX Compiler for release minsize builds" FORCE)
    SET (CMAKE_CXX_FLAGS_RELEASE        "-O4 -DNDEBUG" CACHE STRING "Flags for CXX Compiler for release builds")
    SET (CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g" CACHE STRING "Flags for CXX Compiler for release builds with debug info" FORCE)
    
ENDIF()



#Find Cuda
#find_package(CUDA REQUIRED)

find_package(Eigen3 REQUIRED)
set(EIGEN_INCLUDE_DIR ${EIGEN3_INCLUDE_DIR})

#Searching Boost
set(Boost_ADDITIONAL_VERSIONS "1.46" "1.46.0" "1.46.1" "1.45" "1.45.0" "1.44" "1.44.0" "1.42" "1.42.0" "1.41.0" "1.41" "1.40.0" "1.40" "1.39.0" "1.39" "1.38.0" "1.38" "1.37.0" "1.37" )
set(BOOST_COMPONENTS system timer chrono filesystem)
find_package(Boost COMPONENTS ${BOOST_COMPONENTS} REQUIRED)
if (NOT Boost_FOUND)
	# Try again with the other type of libs
	if(Boost_USE_STATIC_LIBS)
		set(Boost_USE_STATIC_LIBS)
	else()
		set(Boost_USE_STATIC_LIBS ON)
	endif()
	find_package(Boost COMPONENTS ${BOOST_COMPONENTS} QUIET)
endif()

#Find Python

find_package(PythonLibs REQUIRED)


#set common includes for all projects which need them

 set(common_files_inc
	
 )
 set(common_files_src
	
 )

add_subdirectory(Projects/CWrapperTest)

