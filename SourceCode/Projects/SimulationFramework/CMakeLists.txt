#The CMake Minimum version that is required. 
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)


MARK_AS_ADVANCED( SIMULATION_FRAMEWORK_BUILD_GUI )
set(SIMULATIONFRAMEWORK_BUILD_GUI OFF CACHE BOOL "Build the GUI version of SimulationFramework")

MARK_AS_ADVANCED( SIMULATION_FRAMEWORK_BUILD_NOGUI )
set(SIMULATIONFRAMEWORK_BUILD_NOGUI ON CACHE BOOL "Build the NOGUI version of SimulationFramework")

MARK_AS_ADVANCED( SIMULATION_FRAMEWORK_BUILD_MPI )
set(SIMULATIONFRAMEWORK_BUILD_MPI OFF CACHE BOOL "Build the MPI version of SimulationFramework")

MARK_AS_ADVANCED( SIMULATIONFRAMEWORK_BUILD_SIMCONVERTER )
set(SIMULATIONFRAMEWORK_BUILD_SIMCONVERTER ON CACHE BOOL "Build the SimFileConverter for the simulation files of the framework")


# Set the makefile build type (only for make file based generators)
MARK_AS_ADVANCED( CMAKE_BUILD_TYPE )
set(CMAKE_BUILD_TYPE Release CACHE STRING "The build type for makefile based generators")

IF(${CMAKE_GENERATOR} MATCHES "Unix Makefiles")
        SET(SimulationFrameworkName  "SimulationFramework ${CMAKE_BUILD_TYPE}")
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
		SET(CMAKE_CXX_FLAGS "-std=c++11 -ftree-vectorize -ftree-vectorizer-verbose=0 -fmessage-length=0 -Wno-enum-compare" CACHE STRING "Flags for CXX Compiler" FORCE)
		SET(CMAKE_CXX_FLAGS_DEBUG          "-g -fsanitize=address" CACHE STRING "Flags for CXX Compiler for debug builds" FORCE)
		set(GCCFILTER_TOOL OFF CACHE BOOL "Use the perl script gccfilter , http://www.mixtion.org/gccfilter/, this tool gets appended to the gcc command" )
		MARK_AS_ADVANCED(GCCFILTER_TOOL)
    
	ELSEIF ( ${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang" )

		MESSAGE(STATUS "Setting Values for Clang")
		SET(MYPROJECT_SET_COMPILER_FLAGS_INTERNAL TRUE CACHE INTERNAL "x" FORCE)
		SET(CMAKE_C_FLAGS                "-Wall" CACHE STRING "Flags for C Compiler" FORCE)
		SET(CMAKE_C_FLAGS_DEBUG          "-g" CACHE STRING "Flags for C Compiler for debug builds" FORCE)
		SET(CMAKE_C_FLAGS_MINSIZEREL     "-Os -DNDEBUG" CACHE STRING "Flags for C Compiler for release minsize builds" FORCE)
		SET(CMAKE_C_FLAGS_RELEASE        "-O4 -DNDEBUG" CACHE STRING "Flags for C Compiler for release builds" FORCE)
		SET(CMAKE_C_FLAGS_RELWITHDEBINFO "-O2 -g" CACHE STRING "Flags for C Compiler for release builds with debug info" FORCE)

		SET (CMAKE_CXX_FLAGS             "-std=c++11 -ferror-limit=50 -Weverything -Wno-extra-semi -Wno-documentation -pedantic -Wno-conversion -Wno-comment -Wno-unused-parameter -Wno-deprecated-register -Wno-float-equal -Wno-switch -fdiagnostics-show-template-tree -Wno-c++98 -Wno-c++98-compat-pedantic -Wno-deprecated" CACHE STRING "" FORCE)
		#SET(CMAKE_CXX_FLAGS                "std=c++y -ferror-limit=50 -w" CACHE STRING "Flags for CXX Compiler" FORCE)
		SET(CMAKE_CXX_FLAGS_DEBUG          "-g" CACHE STRING "Flags for CXX Compiler for debug builds" FORCE)
		SET(CMAKE_CXX_FLAGS_MINSIZEREL     "-Os -DNDEBUG" CACHE STRING "Flags for CXX Compiler for release minsize builds" FORCE)
		SET(CMAKE_CXX_FLAGS_RELEASE        "-O4 -DNDEBUG" CACHE STRING "Flags for CXX Compiler for release builds" FORCE)
		SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g" CACHE STRING "Flags for CXX Compiler for release builds with debug info" FORCE)
        
        SET(CMAKE_AR      "/usr/bin/local/llvm-ar" CACHE STRING "archiver" FORCE )
        SET(CMAKE_LINKER  "/usr/bin/local/llvm-ld" CACHE STRING "linker tool" FORCE )
        SET(CMAKE_NM      "/usr/bin/local/llvm-nm" CACHE STRING "nm tool" FORCE )
        SET(CMAKE_OBJDUMP "/usr/bin/local/llvm-objdump" CACHE STRING "objdump tool" FORCE )
        SET(CMAKE_RANLIB  "/usr/bin/local/llvm-ranlib" CACHE STRING "ranlib tool" FORCE )
        
	ENDIF()
ENDIF()




SET(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../CMakeModules/;${CMAKE_MODULE_PATH}")
MESSAGE(STATUS "Module path is now set to: " ${CMAKE_MODULE_PATH} )

INCLUDE(PrintListMacro)


#Add some multithreaded build support =====================================================================================================
MARK_AS_ADVANCED(MULTITHREADED_BUILD)
SET(MULTITHREADED_BUILD 12 CACHE STRING "How many threads are used to build the project")
if(MULTITHREADED_BUILD AND NOT MULTITHREADED_BUILD_SET)
	SET(MULTITHREADED_BUILD_SET TRUE CACHE INTERNAL "internal check if we set the multithread stuff already" FORCE)
	if(${CMAKE_GENERATOR} MATCHES "Unix Makefiles")
            SET(CMAKE_MAKE_PROGRAM "${CMAKE_MAKE_PROGRAM} -j${MULTITHREADED_BUILD}")
            MESSAGE(STATUS "Added arguments to CMAKE_BUILD_TOOL: ${CMAKE_MAKE_PROGRAM}")
    elseif(MSVC)
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
      MESSAGE(STATUS "Added parallel build arguments to CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
    endif()
endif()
# ========================================================================================================================================

#Find GCCFilter Tool
if(GCCFILTER_TOOL)
    FIND_PACKAGE(Gccfilter)
	if(GCCFILTER_FOUND)
	    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${GCCFILTER_BIN} ${GCCFILTER_ADDITIONAL_FLAGS}")
	endif()
else()
		set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "")
endif()


#Find Doxygen
FIND_PACKAGE(Doxygen)

MARK_AS_ADVANCED( SIMULATIONFRAMEWORK_USE_CUDA )
SET(SIMULATIONFRAMEWORK_USE_CUDA OFF CACHE BOOL "USE CUDA Support for the SimulationFramework")

if(SIMULATIONFRAMEWORK_USE_CUDA)
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

	set(CUDA_COMMON_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../CudaModern/CommonSource CACHE PATH "Path to the cuda common source directory!")
	MARK_AS_ADVANCED(CUDA_COMMON_SOURCE_DIR)

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
set(Boost_ADDITIONAL_VERSIONS "1.46" "1.46.0" "1.46.1" "1.45" "1.45.0" "1.44" "1.44.0" "1.42" "1.42.0" "1.41.0" "1.41" "1.40.0" "1.40" "1.39.0" "1.39" "1.38.0" "1.38" "1.37.0" "1.37" )
# Components that need linking (NB does not include header-only components like bind)
set(BOOST_COMPONENTS thread date_time system filesystem timer chrono serialization)
find_package(Boost COMPONENTS ${BOOST_COMPONENTS} REQUIRED)
if (NOT Boost_FOUND)
    # Try again with the other type of libs
    if(Boost_USE_STATIC_LIBS)
        SET(Boost_USE_STATIC_LIBS)
    else()
        SET(Boost_USE_STATIC_LIBS ON)
    endif()
    find_package(Boost COMPONENTS ${OGRE_BOOST_COMPONENTS} REQUIRED)
endif()

# ===============================================================================================================================================================


# Searching STLSoft =============================================================================================================================================
# We are using boost::timer now
 #FIND_PACKAGE(STLSoft REQUIRED)
# ===============================================================================================================================================================

# Searching TICPP =============================================================================================================================================
#FIND_PACKAGE(Ticpp REQUIRED)
#PRINTLIST( "TICPP_LIBRARY:" "${TICPP_LIBRARY}")
#PRINTLIST( "TICPP_LIBRARIES:" "${TICPP_LIBRARIES}")
# ===============================================================================================================================================================

# Searching Assimp ==================================
FIND_PACKAGE(Assimp REQUIRED 3)
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





# DEFINE if we should use the GPU for the simulation or not!
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${MYPROJECT_CXX_FLAGS}")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${MYPROJECT_CXX_FLAGS}")
MESSAGE(STATUS "Added ${MYPROJECT_CXX_FLAGS} to CMAKE_CXX and CMAKE_C_FLAGS: ${CMAKE_CXX_FLAGS} and ${CMAKE_C_FLAGS}")

#set common includes for all projects which need them


set(COMMONSOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/CommonSource")

if(SIMULATIONFRAMEWORK_BUILD_GUI)
add_subdirectory(Projects/GeneralRigidBodySimulation)
endif()

if(SIMULATIONFRAMEWORK_BUILD_NOGUI)
add_subdirectory(Projects/GeneralRigidBodySimulationNoGUI)
endif()

if(SIMULATIONFRAMEWORK_BUILD_MPI)
add_subdirectory(Projects/GeneralRigidBodySimulationMPI)
endif()

if(SIMULATIONFRAMEWORK_BUILD_SIMCONVERTER)
add_subdirectory(Projects/Converter)
endif()



# Copy Media Files to Project folder
# message(STATUS "Copying Media Content to " ${CMAKE_BINARY_DIR}/Projects/)
# file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/Projects/media DESTINATION ${CMAKE_BINARY_DIR}/Projects/)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Projects/media
        DESTINATION ${CMAKE_BINARY_DIR}/Projects/
        CONFIGURATIONS Debug Release
)

