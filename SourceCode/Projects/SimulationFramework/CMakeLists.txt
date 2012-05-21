# Set the makefile build type (only for make file based generators)
MARK_AS_ADVANCED( CMAKE_BUILD_TYPE )
set(CMAKE_BUILD_TYPE Release CACHE STRING "The build type for makefile based generators")



#The name of the project ====================================================================
PROJECT(SimulationFramework)

#The CMake Minimum version that is required. The FindCUDA script
#is distributed since version 2.8
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)


set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../CMakeModules/;${CMAKE_MODULE_PATH}")
message(STATUS "Module path is now set to: " ${CMAKE_MODULE_PATH} )

include(PrintListMacro)

set(CMAKE_MAKE_PROGRAM_INTERNAL ${CMAKE_MAKE_PROGRAM} CACHE STRING "internal")



#Add some multithreaded build support =====================================================================================================
MARK_AS_ADVANCED(MULTITHREADED_BUILD)
set(MULTITHREADED_BUILD 12 CACHE STRING "How many threads are used to build the project")
if(MULTITHREADED_BUILD)
    if(${CMAKE_GENERATOR} MATCHES "Unix Makefiles")
            set(CMAKE_MAKE_PROGRAM "${CMAKE_MAKE_PROGRAM_INTERNAL} -j${MULTITHREADED_BUILD}")
            message(STATUS "Added arguments to CMAKE_BUILD_TOOL: ${CMAKE_MAKE_PROGRAM}")
    elseif(MSVC)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
      message(STATUS "Added parallel build arguments to CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
    endif()
endif()
# ========================================================================================================================================


#Find Doxygen
FIND_PACKAGE(Doxygen)

# Find CUDA! =============================================================================================================================
# Set the flags
SET(MYPROJECT_CUDA_PROPAGATE_HOST_FLAGS ON CACHE BOOL "Propagate C/CXX Flags and friends to the host compiler in NVCC via -Xompile")
# Set the architecture for the grafics card!
SET(MYPROJECT_CUDA_NVCC_FLAGS
"-gencode=arch=compute_20,code=\\\"sm_20,compute_20\\\""
"--ptxas-options=-v"
CACHE VALUE "Semicolon delimited several arguments for the NVCC compiler")
SET(MYPROJECT_CUDA_NVCC_FLAGS_DEBUG
"-g"
"-G"
CACHE VALUE "Semicolon delimited several arguments for the NVCC compiler")
SET(MYPROJECT_CUDA_NVCC_FLAGS_RELEASE "-O2"
CACHE VALUE "Semicolon delimited several arguments for the NVCC compiler")

FIND_PACKAGE(CUDA)

if(CUDA_FOUND)
   set(CUDA_SUPPORT 1) #Gets written to the config file!

   SET(CUDA_VERBOSE_BUILD ON)
   SET(CUDA_NVCC_FLAGS           "${MYPROJECT_CUDA_NVCC_FLAGS}"            CACHE INTERNAL "Internal flags do not edit" FORCE)
   SET(CUDA_NVCC_FLAGS_DEBUG     "${MYPROJECT_CUDA_NVCC_FLAGS_DEBUG}"      CACHE INTERNAL "Internal flags do not edit" FORCE)
   SET(CUDA_NVCC_FLAGS_RELEASE   "${MYPROJECT_CUDA_NVCC_FLAGS_RELEASE}"    CACHE INTERNAL "Internal flags do not edit" FORCE)
   SET(CUDA_PROPAGATE_HOST_FLAGS "${MYPROJECT_CUDA_PROPAGATE_HOST_FLAGS}"  CACHE INTERNAL "Internal flags do not edit" FORCE)

   set(CUDA_COMMON_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../CudaFramework/CommonSource CACHE PATH "Path to the cuda common source directory!")
   MARK_AS_ADVANCED(CUDA_COMMON_SOURCE_DIR)

else(CUDA_FOUND)
   set(CUDA_SUPPORT 0) #Gets written to the config file!
endif(CUDA_FOUND)


# ===============================================================================================================================================================

#Find Ogre ======================================================================================================================================================

if(WIN32)
	set(CMAKE_MODULE_PATH "$ENV{OGRE_HOME}/CMake/;${CMAKE_MODULE_PATH}")
endif(WIN32)

if(UNIX)
	set(CMAKE_MODULE_PATH "/usr/local/lib/OGRE/cmake/;${CMAKE_MODULE_PATH}")
endif(UNIX)

if(MSVC10)
   string(REGEX MATCH ".*(msvc10).*" MATCH_MSV10 $ENV{OGRE_HOME})
   if(NOT MATCH_MSV10)
     message(FATAL_ERROR "You need strictly to define the environmenet variable OGRE_HOME=$ENV{OGRE_HOME} to set to the msvc10 directory, to be able to build with MSVC10")
   else(NOT MATCH_MSV10)
      message(STATUS "Ogre envrionement variable set to OGRE_HOME=$ENV{OGRE_HOME}")
   endif(NOT MATCH_MSV10)
endif(MSVC10)

if(MSVC90)
   string(REGEX MATCH ".*(msvc9).*" MATCH_MSV90 $ENV{OGRE_HOME})
   if(NOT MATCH_MSV90)
     message(FATAL_ERROR "You need strictly to define the environmenet variable OGRE_HOME=$ENV{OGRE_HOME} to set to the msvc9 directory, to be able to build with MSVC90")
   else(NOT MATCH_MSV90)
     message(STATUS "Ogre envrionement variable set to OGRE_HOME=$ENV{OGRE_HOME}")
   endif(NOT MATCH_MSV90)
endif(MSVC90)

find_package(OIS REQUIRED)
find_package(OGRE REQUIRED)
#find_package(CEGUI REQUIRED)
# ===============================================================================================================================================================

#Define TooN ====================================================================================================================================================
#find_package(TOON REQUIRED)
# ===============================================================================================================================================================

#Define Eigen ===================================================================================================================================================
find_package(Eigen3 REQUIRED)
set(EIGEN_INCLUDE_DIR ${EIGEN3_INCLUDE_DIR})
# ===============================================================================================================================================================

#Define Lapack
#find_package(SimpleLAPACK REQUIRED)
# ===============================================================================================================================================================

# Find Boost ====================================================================================================================================================
message(STATUS "BOOST_ROOT is set to: $ENV{BOOST_ROOT}")
if (WIN32 OR APPLE)
	set(Boost_USE_STATIC_LIBS TRUE)
else ()
	# Statically linking boost to a dynamic Ogre build doesn't work on Linux 64bit
	set(Boost_USE_STATIC_LIBS ${OGRE_STATIC})
endif ()
if (APPLE)
	if(OGRE_BUILD_PLATFORM_IPHONE)
		set(Boost_COMPILER "-xgcc42")
	endif()
endif()
set(Boost_ADDITIONAL_VERSIONS "1.46" "1.46.0" "1.46.1" "1.45" "1.45.0" "1.44" "1.44.0" "1.42" "1.42.0" "1.41.0" "1.41" "1.40.0" "1.40" "1.39.0" "1.39" "1.38.0" "1.38" "1.37.0" "1.37" )
# Components that need linking (NB does not include header-only components like bind)
set(OGRE_BOOST_COMPONENTS thread date_time system filesystem)
find_package(Boost COMPONENTS ${OGRE_BOOST_COMPONENTS} QUIET)
if (NOT Boost_FOUND)
	# Try again with the other type of libs
	if(Boost_USE_STATIC_LIBS)
		set(Boost_USE_STATIC_LIBS)
	else()
		set(Boost_USE_STATIC_LIBS ON)
	endif()
	find_package(Boost COMPONENTS ${OGRE_BOOST_COMPONENTS} QUIET)
endif()
find_package(Boost QUIET)
# ===============================================================================================================================================================


# Searching STLSoft =============================================================================================================================================
if(CUDA_FOUND)
   FIND_PACKAGE(STLSoft REQUIRED)
endif(CUDA_FOUND)
# ===============================================================================================================================================================

# Searching STLSoft =============================================================================================================================================
FIND_PACKAGE(TinyXML REQUIRED)
# ===============================================================================================================================================================

# Searching Assimp ==================================
FIND_PACKAGE(Assimp REQUIRED)
# ===================================================



# Searching OZCollide or OPCODE (COLLISION LIBRARY) ==================================
SET(MYPROJECT_COLLISION_LIBRARY "ozcollide" CACHE STRING "Choose between 'ozcollide' and 'opcode' for the collision detection...")
MARK_AS_ADVANCED(MYPROJECT_COLLISION_LIBRARY)

STRING(COMPARE EQUAL ${MYPROJECT_COLLISION_LIBRARY}  "opcode" USE_OPCODE)
if(USE_OPCODE)
FIND_PACKAGE(Opcode REQUIRED)
SET(COLLISIONLIB_INCLUDE_DIR ${OPCODE_INCLUDE_DIR})
SET(COLLISIONLIB_LIB_REL ${OPCODE_LIB_REL})
SET(COLLISIONLIB_LIB_DBG ${OPCODE_LIB_DBG})
SET(COLLISIONLIB_REL ${OPCODE_REL})
SET(COLLISIONLIB_DBG ${OPCODE_DBG})
SET(COLLISIONLIB_LIBRARIES ${OPCODE_LIBRARIES})
SET(USE_OPCODE 1)
SET(OPCODE_PRECISION "double")
else()
SET(USE_OPCODE 0)
endif()

STRING(COMPARE EQUAL ${MYPROJECT_COLLISION_LIBRARY}  "ozcollide" USE_OZCOLLIDE)
if(USE_OZCOLLIDE)
#MESSAGE(FATAL_ERROR "OZCOLIDE has not been really used!")
# Searching OZCollide ==================================
FIND_PACKAGE(OZCollide REQUIRED)
# ===================================================
SET(COLLISIONLIB_INCLUDE_DIR ${OZCOLLIDE_INCLUDE_DIR})
SET(COLLISIONLIB_LIB_REL ${OZCOLLIDE_LIB_REL})
SET(COLLISIONLIB_LIB_DBG ${OZCOLLIDE_LIB_DBG})
SET(COLLISIONLIB_REL ${OZCOLLIDE_REL})
SET(COLLISIONLIB_DBG ${OZCOLLIDE_DBG})
SET(COLLISIONLIB_LIBRARIES ${OZCOLLIDE_LIBRARIES})
SET(USE_OZCOLLIDE 1)
else()
SET(USE_OZCOLLIDE 0)
endif()





# DEFINE if we should use the GPU for the simulation or not!
#






#set common includes for all projects which need them
add_subdirectory(CommonSource)
PRINTLIST("Common File Includes are:" "${COMMON_FILES_INC}")
PRINTLIST("Common File Sources are:" "${COMMON_FILES_SRC}")
set(COMMON_SOURCE_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/CommonSource/include")


add_subdirectory(Projects/GeneralRigidBodySimulation)

# Copy Media Files to Project folder
# message(STATUS "Copying Media Content to " ${CMAKE_BINARY_DIR}/Projects/)
# file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/Projects/media DESTINATION ${CMAKE_BINARY_DIR}/Projects/)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Projects/media
		DESTINATION ${CMAKE_BINARY_DIR}/Projects/
		CONFIGURATIONS Debug Release
)
