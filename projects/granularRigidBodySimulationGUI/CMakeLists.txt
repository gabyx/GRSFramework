

PROJECT(GRSFSimGUI)

message(STATUS "==============================Project: ${PROJECT_NAME} ==================================")
include(PrintListMacro)


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

# take care Ogre loads boost too save libraries
set(Boost_LIBRARIES_SAVE ${Boost_LIBRARIES} )

find_package(OIS)
find_package(OGRE)
#find_package(CEGUI REQUIRED)

#foreach(i ${Boost_LIBRARIES})
#message(STATUS "remove: " ${i})
#list(REMOVE_ITEM Boost_LIBRARIES_SAVE ${i})
#endforeach()
set(Boost_LIBRARIES ${Boost_LIBRARIES} ${Boost_LIBRARIES_SAVE})
#message(WARNING "${Boost_LIBRARIES}")
# ===============================================================================================================================================================


# Check if OGRE and OIS has been found!!
if(NOT OGRE_FOUND OR NOT OIS_FOUND)
   MESSAGE(ERROR "For this project to build, OGRE and OIS library need to be found!")
else()
   MESSAGE(STATUS "OGRE and OIS Library have been found!")
endif()



#Set Source Code



SET( APP_INC
    ${PROJECT_SOURCE_DIR}/include/GRSF/app/App.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/app/MenuMouse.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/app/OrbitCamera.hpp
)
SET( APP_SRC
    ${PROJECT_SOURCE_DIR}/src/GRSF/app/App.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/app/MenuMouse.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/app/OrbitCamera.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/app/main.cpp

)

SET(SINGELTON_INC
    ${PROJECT_SOURCE_DIR}/include/GRSF/singeltons/contexts/GuiContext.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/singeltons/contexts/InputContext.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/singeltons/contexts/RenderContext.hpp
)

SET( SINGELTON_SRC
    ${PROJECT_SOURCE_DIR}/src/GRSF/singeltons/contexts/GuiContext.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/singeltons/contexts/InputContext.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/singeltons/contexts/RenderContext.cpp
)

SET( STATES_INC
    ${PROJECT_SOURCE_DIR}/include/GRSF/states/AppState.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/states/AppStateManager.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/states/SimulationState.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/states/PlaybackState.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/states/VideoDropper.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/states/simulationManager/SimulationManagerBase.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/states/simulationManager/SimulationManagerGUI.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/states/simulationManager/PlaybackManagerBase.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/states/simulationManager/PlaybackManager.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/states/simulationManager/PlaybackLoader.hpp
)
SET(STATES_SRC
    ${PROJECT_SOURCE_DIR}/src/GRSF/states/simulationManager/PlaybackManager.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/states/simulationManager/SimulationManagerGUI.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/states/AppStateManager.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/states/SimulationState.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/states/PlaybackState.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/states/simulationManager/SimulationManagerBase.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/states/simulationManager/PlaybackManagerBase.cpp
)

SET(COMMON_INC
    ${PROJECT_SOURCE_DIR}/include/GRSF/common/TypeDefs.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/common/OgreMeshExtraction.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/common/AxisObject.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/common/DynamicLines.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/common/DynamicRenderable.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/common/OgrePointCloud.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/common/CommonFunctionsOgre.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/common/LogDefines.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/common/Asserts.hpp
)
SET(COMMON_SRC
    ${PROJECT_SOURCE_DIR}/src/GRSF/common/OgreMeshExtraction.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/common/AxisObject.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/common/DynamicLines.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/common/DynamicRenderable.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/common/OgrePointCloud.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/common/CommonFunctionsOgre.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/common/TypeDefs.cpp
)

SET(DYNAMICS_INC
     ${PROJECT_SOURCE_DIR}/include/GRSF/dynamics/general/DynamicsSystemGUI.hpp
)
SET(DYNAMICS_SRC
     ${PROJECT_SOURCE_DIR}/src/GRSF/dynamics/general/DynamicsSystemGUI.cpp
)

SET(SYSTEMS_INC
    ${PROJECT_SOURCE_DIR}/include/GRSF/systems/SceneParserGUI.hpp
)

SET(SYSTEMS_SRC
)

# COMMON FILES INCLUDE ====================================================================
INCLUDE(IncludeModulesGRSFramework)

INCLUDE_SIMULATION_FRAMEWORK_GUI( GLOBAL_COMMON_SRC  GLOBAL_COMMON_INC GLOBAL_COMMON_INC_DIR ${GRSF_COMMON_SOURCE_DIR})

# ===========================================================================================

# CUDA SOURCE INCLUDES ====================================================================
if(GRSF_USE_CUDA)
MESSAGE(ERROR "CUDA does not work, write library to include!")
INCLUDE(IncludeModulesCUDAModern)

# Configure File for the BUILD!
string(REPLACE "." "" CUDA_VERSION_INTEGER ${CUDA_VERSION})

# GPU Modules (JOR/SOR Lambda Dense Modules)
INCLUDE_PROX_CUDA(ProxGPU_SRC ProxGPU_INC CUDA_COMMON_INCLUDE_DIR ${CUDA_COMMON_SOURCE_DIR})
INCLUDE_VECTOR_ADD_CUDA(VectorAddGPU_SRC VectorAddGPU_INC CUDA_COMMON_INCLUDE_DIR ${CUDA_COMMON_SOURCE_DIR})
INCLUDE_MATRIX_VECTOR_MULT_CUDA(MatrixVectorMultGPU_SRC MatrixVectorMultGPU_INC CUDA_COMMON_INCLUDE_DIR ${CUDA_COMMON_SOURCE_DIR})
INCLUDE_GENERAL_EXTERN_CUDA(GeneralCommonFiles_SRC GeneralCommonFiles_INC CUDA_COMMON_INCLUDE_DIR ${CUDA_COMMON_SOURCE_DIR})

# GPU Modules (JOR Velocity Modules)
INCLUDE_JORPROX_VELOCITY_MODULE_EXTERN_CUDA( JorProxVelocity_SRC  JorProxVelocity_INC CUDA_COMMON_INCLUDE_DIR ${CUDA_COMMON_SOURCE_DIR})

SET(CUDA_SRC
   ${ProxGPU_SRC} ${VectorAddGPU_SRC} ${MatrixVectorMultGPU_SRC} ${GeneralCommonFiles_SRC} ${JorProxVelocity_SRC}
)
SET(CUDA_INC
   ${ProxGPU_INC} ${VectorAddGPU_INC} ${MatrixVectorMultGPU_INC} ${GeneralCommonFiles_INC} ${JorProxVelocity_INC}
)
else()
# Include nothing
endif()
# ===========================================================================================

#Configure ApproxMVBB =======================================================================
include(${ApproxMVBB_CMAKE_DIR}/DefineApproxMVBBSources.cmake)

#write config file for this source code
#set(ApproxMVBB_USE_DIFFERENT_HEADERS TRUE)
#set(ApproxMVBB_AssertionDebug_INCLUDE_FILE     "ApproxMVBB/Common//Asserts.hpp")
#set(ApproxMVBB_Exception_INCLUDE_FILE          "ApproxMVBB/Common//Exception.hpp")
#set(ApproxMVBB_Platform_INCLUDE_FILE           "ApproxMVBB/Common/Platform.hpp")
#set(ApproxMVBB_StaticAssert_INCLUDE_FILE       "ApproxMVBB/Common/StaticAssert.hpp")
#set(ApproxMVBB_TypeDefs_INCLUDE_FILE           "ApproxMVBB/Common/TypeDefs.hpp" )
#set(ApproxMVBB_MyMatrixTypeDefs_INCLUDE_FILE   "ApproxMVBBWrapper/Common/MyMatrixTypeDefs.hpp")
#set(ApproxMVBB_AABB_INCLUDE_FILE               "GRSF/Dynamics/Collision/Geometry/AABB.hpp")
#set(ApproxMVBB_OOBB_INCLUDE_FILE               "GRSF/Dynamics/Collision/Geometry/OOBB.hpp" )

# Define all MVBB Source files
INCLUDE_ALL_ApproxMVBB_SOURCE(  ApproxMVBB_SRC 
                                ApproxMVBB_INC 
                                ApproxMVBB_INC_DIRS 
                                ApproxMVBB_DEPENDING_TARGETS
                                ${ApproxMVBB_ROOT_DIR} ${PROJECT_BINARY_DIR} )
# ===========================================================================================

# WRITE CONFIGURATION FILE
SET(CONFIG_INC ${PROJECT_BINARY_DIR}/include/GRSF/configFiles/ConfigureFile.hpp)
configure_file (
  ${PROJECT_SOURCE_DIR}/include/GRSF/configFiles/ConfigureFile.hpp.in
  ${CONFIG_INC}
)
#=========================


SET(SOURCE_FILES
    ${APP_SRC}
    ${SINGELTON_SRC}
    ${STATES_SRC}
    ${COMMON_SRC}
    ${DYNAMICS_SRC}
    ${SYSTEMS_SRC}
    ${CUDA_SRC}
    ${GLOBAL_COMMON_SRC}
    
    ${ApproxMVBB_SRC}
)

SET(INCLUDE_FILES
    ${CONFIG_INC}
    ${APP_INC}
    ${SINGELTON_INC}
    ${STATES_INC}
    ${COMMON_INC}
    ${DYNAMICS_INC}
    ${SYSTEMS_INC}
    ${CUDA_INC}
    ${GLOBAL_COMMON_INC}
    
    ${ApproxMVBB_INC}
)


#Set CEGUI Libraries
#SET(CEGUI_LIBS CEGUIBase CEGUIOgreRenderer)

SET(CMAKE_DEBUG_POSTFIX "_d")

#include directories
set(INCLUDE_DIRS

    ${GRSF_DEP_INCLUDE_DIRS}
    
    ${OGRE_INCLUDE_DIRS}
    ${OGRE_Overlay_INCLUDE_DIR}
    
    ${OIS_INCLUDE_DIRS}
    #${CEGUI_INCLUDE_DIR}

    ${PROJECT_BINARY_DIR}/include/
    ${PROJECT_SOURCE_DIR}/include/

    ${GLOBAL_COMMON_INC_DIR}
    ${CUDA_COMMON_INCLUDE_DIR}
    ${ApproxMVBB_INC_DIRS}
)

include_directories(${INCLUDE_DIRS})
#PRINTLIST("Include directories are:" "${INCLUDE_DIRS}")



#link directories
set(LINK_DIRS
    #"${CEGUI_LIBRARY_DIR}"
    ${GRSF_DEP_LIBRARY_DIRS}
)
link_directories(${LINK_DIRS})
PRINTLIST( "Link directories are:" "${LINK_DIRS}")



set(LINK_LIBS 
    ${GRSF_DEP_LIBRARIES}
    
    ${OGRE_LIBRARIES}
    ${OGRE_Overlay_LIBRARIES}
    ${OIS_LIBRARIES} 
    ${CUDA_cublas_LIBRARY}
)
PRINTLIST( "Linked libraries are:" "${LINK_LIBS}")


message(STATUS "Add executable:  ${PPROJECT_NAME}")
if(GRSF_USE_CUDA)
  CUDA_ADD_EXECUTABLE(${PROJECT_NAME}  ${SOURCE_FILES} ${INCLUDE_FILES} )
else()
  ADD_EXECUTABLE(${PROJECT_NAME}  ${SOURCE_FILES} ${INCLUDE_FILES} )
endif()
target_link_libraries(${PROJECT_NAME} ${LINK_LIBS} )

set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin/
    RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_BINARY_DIR}/bin/debug
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_BINARY_DIR}/bin/release
)

# Add dependency 
add_dependencies(${PROJECT_NAME}  AddMediaFilesToBuildFolder ${ApproxMVBB_DEPENDING_TARGETS})

# Add target for copying Resources Release and Debug
SET(ResourceDir ${CMAKE_CURRENT_SOURCE_DIR}/resources)
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND 
          ${CMAKE_COMMAND}
          -DFolder=$<$<CONFIG:debug>:${ResourceDir}/debug>$<$<CONFIG:release>:${ResourceDir}/release>
          -DDestination=$<$<CONFIG:debug>:${PROJECT_BINARY_DIR}/bin/Debug>$<$<CONFIG:release>:${PROJECT_BINARY_DIR}/bin/release>
          -P ${MODULE_FOLDER}/CopyFiles.cmake
)

message(STATUS "=========================================================================")
