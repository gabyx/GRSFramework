#The CMake Minimum version that is required. 
CMAKE_MINIMUM_REQUIRED(VERSION 3.0)


MARK_AS_ADVANCED( SIMULATION_FRAMEWORK_BUILD_GUI )
set(GRSF_BUILD_GUI ON CACHE BOOL "Build the GUI version of SimulationFramework")

MARK_AS_ADVANCED( SIMULATION_FRAMEWORK_BUILD_NOGUI )
set(GRSF_BUILD_NOGUI ON CACHE BOOL "Build the NOGUI version of SimulationFramework")

MARK_AS_ADVANCED( SIMULATION_FRAMEWORK_BUILD_MPI )
set(GRSF_BUILD_MPI ON CACHE BOOL "Build the MPI version of SimulationFramework")

MARK_AS_ADVANCED( GRSF_BUILD_SIMCONVERTER )
set(GRSF_BUILD_SIMCONVERTER ON CACHE BOOL "Build the SimFileConverter for the simulation files of the framework")

MARK_AS_ADVANCED( GRSF_BUILD_TESTBENCH )
set(GRSF_BUILD_TESTBENCH OFF CACHE BOOL "Build the TestBench executables and all tests")

#Detect Build Type if Debug appears in CMAKE_BINARY_DIR path
MARK_AS_ADVANCED( CMAKE_BUILD_TYPE )
STRING(REGEX MATCH ".*(Debug|debug).*" DEBUG_MATCH ${CMAKE_BINARY_DIR} )
if(DEBUG_MATCH)
    MESSAGE(WARNING "Matched Debug build!")
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "The build type for makefile based generators")
ELSE()
    MESSAGE(WARNING "Matched Release build!")
    set(CMAKE_BUILD_TYPE Release CACHE STRING "The build type for makefile based generators")
ENDIF()


SET(MODULE_FOLDER "${CMAKE_CURRENT_SOURCE_DIR}/cmake/" )
SET(CMAKE_MODULE_PATH ${MODULE_FOLDER} ${CMAKE_MODULE_PATH})
MESSAGE(STATUS "Module path is now set to: " ${CMAKE_MODULE_PATH} )


SET(SimulationFrameworkName "GRSFramework")
IF(${CMAKE_GENERATOR} MATCHES "Unix Makefiles")
        SET(SimulationFrameworkName  "${SimulationFrameworkName}-${CMAKE_BUILD_TYPE}")
ENDIF()

MESSAGE(STATUS "Project name is: " ${SimulationFrameworkName})

#The name of the project ====================================================================
PROJECT(${SimulationFrameworkName})

MESSAGE(STATUS "Compiler ID is: " ${CMAKE_CXX_COMPILER_ID})
IF(NOT MYPROJECT_SET_COMPILER_FLAGS_INTERNAL)
	IF(${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
	
		SET(MYPROJECT_SET_COMPILER_FLAGS_INTERNAL TRUE CACHE INTERNAL "x" FORCE)
		MESSAGE(STATUS "Setting Values for GNU")
		SET(CMAKE_C_FLAGS "-fmessage-length=0" CACHE STRING "Flags for C Compiler" FORCE)
		SET(CMAKE_CXX_FLAGS "-std=c++14 -ftree-vectorize -ftree-vectorizer-verbose=0 -fmessage-length=0 -Wno-enum-compare" CACHE STRING "Flags for CXX Compiler" FORCE)
		SET(CMAKE_CXX_FLAGS_DEBUG          "-g -fsanitize=address -fno-omit-frame-pointer -O1" CACHE STRING "Flags for CXX Compiler for debug builds" FORCE)
		set(GCCFILTER_TOOL OFF CACHE BOOL "Use the perl script gccfilter , http://www.mixtion.org/gccfilter/, this tool gets appended to the gcc command" )
		MARK_AS_ADVANCED(GCCFILTER_TOOL)
    
	ELSEIF ( ${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang" )

		MESSAGE(STATUS "Setting Values for Clang")
		SET(MYPROJECT_SET_COMPILER_FLAGS_INTERNAL TRUE CACHE INTERNAL "x" FORCE)
		SET(CMAKE_C_FLAGS                "-Wall -Werror=return-type" CACHE STRING "Flags for C Compiler" FORCE)
		SET(CMAKE_C_FLAGS_DEBUG          "-g" CACHE STRING "Flags for C Compiler for debug builds" FORCE)
		SET(CMAKE_C_FLAGS_MINSIZEREL     "-Os -DNDEBUG" CACHE STRING "Flags for C Compiler for release minsize builds" FORCE)
		SET(CMAKE_C_FLAGS_RELEASE        "-O3 -DNDEBUG" CACHE STRING "Flags for C Compiler for release builds" FORCE)
		SET(CMAKE_C_FLAGS_RELWITHDEBINFO "-O2 -g" CACHE STRING "Flags for C Compiler for release builds with debug info" FORCE)

        #SET (CMAKE_CXX_FLAGS             "-std=c++14 -ferror-limit=50 -Weverything -Wno-extra-semi -Wno-documentation -pedantic -Wno-conversion -Wno-comment -Wno-unused-parameter -Wno-deprecated-register -Wno-float-equal -Wno-switch -fdiagnostics-show-template-tree -Wno-c++98 -Wno-c++98-compat-pedantic -Wno-deprecated" CACHE STRING "" FORCE)
		SET(CMAKE_CXX_FLAGS                "-std=c++14 -ferror-limit=50 -w -fmessage-length=0 -Werror=return-type" CACHE STRING "Flags for CXX Compiler" FORCE)
		SET(CMAKE_CXX_FLAGS_DEBUG          "-g -fsanitize=leak -fsanitize=address -fno-omit-frame-pointer -O1" CACHE STRING "Flags for CXX Compiler for debug builds" FORCE)
		SET(CMAKE_CXX_FLAGS_MINSIZEREL     "-Os -DNDEBUG" CACHE STRING "Flags for CXX Compiler for release minsize builds" FORCE)
		SET(CMAKE_CXX_FLAGS_RELEASE        "-O3 -DNDEBUG" CACHE STRING "Flags for CXX Compiler for release builds" FORCE)
		SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g" CACHE STRING "Flags for CXX Compiler for release builds with debug info" FORCE)
        
        SET(CMAKE_AR      "/usr/bin/local/llvm-ar" CACHE STRING "archiver" FORCE )
        SET(CMAKE_LINKER  "/usr/bin/local/llvm-ld" CACHE STRING "linker tool" FORCE )
        SET(CMAKE_NM      "/usr/bin/local/llvm-nm" CACHE STRING "nm tool" FORCE )
        SET(CMAKE_OBJDUMP "/usr/bin/local/llvm-objdump" CACHE STRING "objdump tool" FORCE )
        SET(CMAKE_RANLIB  "/usr/bin/local/llvm-ranlib" CACHE STRING "ranlib tool" FORCE )
        
	ELSEIF ( ${CMAKE_CXX_COMPILER_ID} STREQUAL "Intel" )
        
        MESSAGE(STATUS "Setting Values for Intel")
		SET(MYPROJECT_SET_COMPILER_FLAGS_INTERNAL TRUE CACHE INTERNAL "x" FORCE)
		SET(CMAKE_C_FLAGS                "-w" CACHE STRING "Flags for C Compiler" FORCE)
		SET(CMAKE_C_FLAGS_DEBUG          "-g" CACHE STRING "Flags for C Compiler for debug builds" FORCE)
		SET(CMAKE_C_FLAGS_MINSIZEREL     "-Os -DNDEBUG" CACHE STRING "Flags for C Compiler for release minsize builds" FORCE)
		SET(CMAKE_C_FLAGS_RELEASE        "-O3 -DNDEBUG" CACHE STRING "Flags for C Compiler for release builds" FORCE)
		SET(CMAKE_C_FLAGS_RELWITHDEBINFO "-O2 -g" CACHE STRING "Flags for C Compiler for release builds with debug info" FORCE)

		SET(CMAKE_CXX_FLAGS                "-std=c++11 -w" CACHE STRING "Flags for CXX Compiler" FORCE)
		SET(CMAKE_CXX_FLAGS_DEBUG          "-g" CACHE STRING "Flags for CXX Compiler for debug builds" FORCE)
		SET(CMAKE_CXX_FLAGS_MINSIZEREL     "-Os -DNDEBUG" CACHE STRING "Flags for CXX Compiler for release minsize builds" FORCE)
		SET(CMAKE_CXX_FLAGS_RELEASE        "-O3 -DNDEBUG" CACHE STRING "Flags for CXX Compiler for release builds" FORCE)
		SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g" CACHE STRING "Flags for CXX Compiler for release builds with debug info" FORCE)
        
    ENDIF()
ENDIF()



INCLUDE(PrintListMacro)

#Add some multithreaded build support =====================================================================================================
MARK_AS_ADVANCED(MULTITHREADED_BUILD)
SET(MULTITHREADED_BUILD ON CACHE BOOL "Parallel build with as many threads as possible!")
if(MULTITHREADED_BUILD)
	if(${CMAKE_GENERATOR} MATCHES "Unix Makefiles")
             file(COPY ${MODULE_FOLDER}/parallelmake.sh DESTINATION ${PROJECT_BINARY_DIR}
                FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
                NO_SOURCE_PERMISSIONS
            )
            SET(CMAKE_MAKE_PROGRAM "${PROJECT_BINARY_DIR}/parallelmake.sh")
            MESSAGE(STATUS "Set make program to ${PROJECT_BINARY_DIR}/parallelmake.sh")
    elseif(MSVC)
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}" "/MP")
      MESSAGE(STATUS "Added parallel build arguments to CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
    endif()
endif()
# ========================================================================================================================================


#Find Doxygen
FIND_PACKAGE(Doxygen)

MARK_AS_ADVANCED( GRSF_USE_CUDA )
SET(GRSF_USE_CUDA OFF CACHE BOOL "USE CUDA Support for the SimulationFramework")

if(GRSF_USE_CUDA)
# Find CUDA (if available)! =============================================================================================================================
	# Set the flags

	IF(WIN32)
			SET(MYPROJECT_CUDA_PROPAGATE_HOST_FLAGS ON CACHE BOOL "Propagate C/CXX Flags and friends to the host compiler in NVCC via -Xompile")
	ENDIF()
	IF(UNIX)
			SET(MYPROJECT_CUDA_PROPAGATE_HOST_FLAGS OFF CACHE BOOL "Propagate C/CXX Flags and friends to the host compiler in NVCC via -Xompile")
	ENDIF()

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

	FIND_PACKAGE(CUDA REQUIRED)

	   set(CUDA_SUPPORT 1) #Gets written to the config file!

	   SET(CUDA_VERBOSE_BUILD ON)
	   SET(CUDA_NVCC_FLAGS           "${MYPROJECT_CUDA_NVCC_FLAGS}"            CACHE INTERNAL "Internal flags do not edit" FORCE)
	   SET(CUDA_NVCC_FLAGS_DEBUG     "${MYPROJECT_CUDA_NVCC_FLAGS_DEBUG}"      CACHE INTERNAL "Internal flags do not edit" FORCE)
	   SET(CUDA_NVCC_FLAGS_RELEASE   "${MYPROJECT_CUDA_NVCC_FLAGS_RELEASE}"    CACHE INTERNAL "Internal flags do not edit" FORCE)
	   SET(CUDA_PROPAGATE_HOST_FLAGS "${MYPROJECT_CUDA_PROPAGATE_HOST_FLAGS}"  CACHE INTERNAL "Internal flags do not edit" FORCE)

	   set(CUDA_COMMON_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../CudaFramework/CommonSource CACHE PATH "Path to the cuda common source directory!")
	   MARK_AS_ADVANCED(CUDA_COMMON_SOURCE_DIR)

else()

	#set(CUDA_COMMON_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../CudaModern/CommonSource CACHE PATH "Path to the cuda common source directory!")
	#MARK_AS_ADVANCED(CUDA_COMMON_SOURCE_DIR)

	set(CUDA_SUPPORT 0) #Gets written to the config file!
endif()


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
    SET(Boost_USE_STATIC_LIBS TRUE)
else ()
    # Statically linking boost to a dynamic Ogre build doesn't work on Linux 64bit
    SET(Boost_USE_STATIC_LIBS ${OGRE_STATIC})
endif ()
if (APPLE)
    if(OGRE_BUILD_PLATFORM_IPHONE)
        SET(Boost_COMPILER "-xgcc42")
    endif()
endif()
set(Boost_ADDITIONAL_VERSIONS "1.58" "1.54")
# Components that need linking (NB does not include header-only components like bind)
set(BOOST_COMPONENTS "system" "thread" "filesystem" "serialization" )
find_package(Boost COMPONENTS ${BOOST_COMPONENTS} REQUIRED)
if (NOT Boost_FOUND)
    # Try again with the other type of libs
    if(Boost_USE_STATIC_LIBS)
        SET(Boost_USE_STATIC_LIBS)
    else()
        SET(Boost_USE_STATIC_LIBS ON)
    endif()
    find_package(Boost COMPONENTS ${BOOST_COMPONENTS} REQUIRED)
endif()
# ===============================================================================================================================================================


# Searching STLSoft =============================================================================================================================================
# We are using boost::timer now
 #FIND_PACKAGE(STLSoft REQUIRED)
# ===============================================================================================================================================================

# Searching PugiXML =============================================================================================================================================
FIND_PACKAGE(PugiXML REQUIRED)
# ===============================================================================================================================================================

# Searching Meta ================================================================================================================================================
FIND_PACKAGE(Meta REQUIRED)
# ===============================================================================================================================================================

# Searching Assimp ==================================
find_package(Assimp 3 REQUIRED)
# ===================================================

# Searching ApproxMVBB Source ==================================

# to link against ApproxMVBB
#find_package(ApproxMVBB)   

# to include all source into the project
if(NOT ApproxMVBB_SEARCH_PATH)
  set(ApproxMVBB_SEARCH_PATH "./../ApproxMVBB" CACHE STRING "ApproxMVBB Repo Dir")
endif()
find_package(ApproxMVBBSource REQUIRED)  
# ===================================================

# Searching OZCollide or OPCODE (COLLISION LIBRARY) ==================================
SET(MYPROJECT_COLLISION_LIBRARY "none" CACHE STRING "Choose between 'ozcollide' and 'opcode' for the collision detection...")
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

  SET(USE_OWN_COLLISION_CODE 0)
  SET(USE_OPCODE 1)
  SET(OPCODE_PRECISION "double")
else()
  SET(USE_OWN_COLLISION_CODE 1)
  SET(USE_OPCODE 0)
endif()

STRING(COMPARE EQUAL ${MYPROJECT_COLLISION_LIBRARY}  "ozcollide" USE_OZCOLLIDE)
if(USE_OZCOLLIDE)
  #MESSAGE(FATAL_ERROR "OZCOLIDE has not been really used!")
  # Searching OZCollide ==================================
  FIND_PACKAGE(OZCollide REQUIRED)
  # ===================================================
  SET(COLLISIONLIB_INCLUDE_DIRS ${OZCOLLIDE_INCLUDE_DIR})
  SET(COLLISIONLIB_LIB_REL ${OZCOLLIDE_LIB_REL})
  SET(COLLISIONLIB_LIB_DBG ${OZCOLLIDE_LIB_DBG})
  SET(COLLISIONLIB_REL ${OZCOLLIDE_REL})
  SET(COLLISIONLIB_DBG ${OZCOLLIDE_DBG})
  SET(COLLISIONLIB_LIBRARIES ${OZCOLLIDE_LIBRARIES})

  SET(USE_OWN_COLLISION_CODE 0)
  SET(USE_OZCOLLIDE 1)
else()
  SET(USE_OWN_COLLISION_CODE 1)
  SET(USE_OZCOLLIDE 0)
endif()


# set common include dir
SET(GRSF_DEP_INCLUDE_DIRS 
    ${Boost_INCLUDE_DIRS}
    
    ${EIGEN_INCLUDE_DIR}
    ${EIGEN_INCLUDE_DIR}/unsupported
    
    ${PUGIXML_INCLUDE_DIR}
    ${Meta_INCLUDE_DIR}
    ${PUGIXML_INCLUDE_DIR}
    ${ASSIMP_INCLUDE_DIR}
    
    ${APPROX_INCLUDE_DIR}
    ${COLLISIONLIB_INCLUDE_DIRS}
)
# set common link libraries
SET(GRSF_DEP_LIBRARIES ${Boost_LIBRARIES} ${ASSIMP_LIBRARIES} ${PUGIXML_LIBRARIES} )
SET(GRSF_DEP_LIBRARY_DIRS ${Boost_LIBRARY_DIR})

# DEFINE if we should use the GPU for the simulation or not!
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${MYPROJECT_CXX_FLAGS}")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${MYPROJECT_CXX_FLAGS}")
MESSAGE(STATUS "Added ${MYPROJECT_CXX_FLAGS} to CMAKE_CXX and CMAKE_C_FLAGS: ${CMAKE_CXX_FLAGS} and ${CMAKE_C_FLAGS}")

#set common includes for all projects which need them
set(GRSF_COMMON_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/common")



include(GetGRSFVersionNumber)
GetGRSFVersionNumber()


if(GRSF_BUILD_GUI)
add_subdirectory(projects/granularRigidBodySimulationGUI)
endif()

if(GRSF_BUILD_NOGUI)
add_subdirectory(projects/granularRigidBodySimulation)
endif()

if(GRSF_BUILD_MPI)
add_subdirectory(projects/granularRigidBodySimulationMPI)
endif()

if(GRSF_BUILD_TESTBENCH)
add_subdirectory(tests/TestBench)
endif()

if(GRSF_BUILD_SIMCONVERTER)
add_subdirectory(projects/converter)
endif()


## Copy Media Files to Project folder (unix symlink, other copy)
if(${UNIX})  
add_custom_target(
    AddMediaFilesToBuildFolder
    COMMAND 
          ${CMAKE_COMMAND}
          -E
          create_symlink
          "${CMAKE_CURRENT_SOURCE_DIR}/projects/media"
          "${CMAKE_BINARY_DIR}/projects/media"
)
else()
add_custom_target(
    AddMediaFilesToBuildFolder
    COMMAND 
          ${CMAKE_COMMAND}
          -E
          copy_directory
          "${CMAKE_CURRENT_SOURCE_DIR}/projects/media"
          "${CMAKE_BINARY_DIR}/projects/media"
)
endif()

