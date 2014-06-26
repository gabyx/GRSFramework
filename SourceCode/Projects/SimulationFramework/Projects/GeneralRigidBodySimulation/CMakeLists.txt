

PROJECT(GeneralRigidBodySimulation)

message(STATUS "==============================Project: ${PROJECT_NAME} ==================================")
include(PrintListMacro)
## EXTREMLY IMPORTANT ISSUES ############################################################################################################################################
# In MSVC 64bit mode, there is no problem with vectorization at all, because everything is aligned nicely!,
# If we move this code to UNIX 64bit it is not anymore like that, the Code of me, is not thoroughly supported for Vectorization in EIGEN
# TODO: To make it fully work with EIGEN_VECTORIZE as PREPROCESSOR MACRO (standart, set see App Log, have made an output!
# in 32bit on Windows and Unix, we need the EIGEN_MAKE_ALLOCATOR_NEW always defined, (almost done) and all std::vector and stuff we need special allocators! not done!
# TO make it still work on UNIX 64bit, turn EIGEN_DONT_ALIGN, for disabling completetly, and  EIGEN_DONT_ALIGN_STATICALLY for partly which still works... (or should)
# http://eigen.tuxfamily.org/dox-devel/TopicPreprocessorDirectives.html
#########################################################################################################################################################################

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

find_package(OIS)
find_package(OGRE)
#find_package(CEGUI REQUIRED)
# ===============================================================================================================================================================


# Check if OGRE and OIS has been found!!
if(NOT OGRE_FOUND OR NOT OIS_FOUND)
   MESSAGE(ERROR "For this project to build, OGRE and OIS library need to be found!")
else()
   MESSAGE(STATUS "OGRE and OIS Library have been found!")
endif()



#Set Source Code

SET(CONFIG_INC ${PROJECT_BINARY_DIR}/ConfigFiles/ConfigureFile.hpp)

SET( APP_INC
    ${PROJECT_SOURCE_DIR}/include/App/App.hpp
    ${PROJECT_SOURCE_DIR}/include/App/MenuMouse.hpp
    ${PROJECT_SOURCE_DIR}/include/App/OrbitCamera.hpp
)
SET( APP_SRC
    ${PROJECT_SOURCE_DIR}/src/App/App.cpp
    ${PROJECT_SOURCE_DIR}/src/App/MenuMouse.cpp
    ${PROJECT_SOURCE_DIR}/src/App/OrbitCamera.cpp
    ${PROJECT_SOURCE_DIR}/src/App/main.cpp

)

SET(SINGELTON_INC
    ${PROJECT_SOURCE_DIR}/include/Singeltons/Contexts/GuiContext.hpp
    ${PROJECT_SOURCE_DIR}/include/Singeltons/Contexts/InputContext.hpp
    ${PROJECT_SOURCE_DIR}/include/Singeltons/Contexts/RenderContext.hpp
)

SET( SINGELTON_SRC
    ${PROJECT_SOURCE_DIR}/src/Singeltons/Contexts/GuiContext.cpp
    ${PROJECT_SOURCE_DIR}/src/Singeltons/Contexts/InputContext.cpp
    ${PROJECT_SOURCE_DIR}/src/Singeltons/Contexts/RenderContext.cpp
)

SET( STATES_INC
    ${PROJECT_SOURCE_DIR}/include/States/AppState.hpp
    ${PROJECT_SOURCE_DIR}/include/States/AppStateManager.hpp
    ${PROJECT_SOURCE_DIR}/include/States/SimulationState.hpp
    ${PROJECT_SOURCE_DIR}/include/States/PlaybackState.hpp
    ${PROJECT_SOURCE_DIR}/include/States/VideoDropper.hpp
    ${PROJECT_SOURCE_DIR}/include/States/SimulationManager/SimulationManagerBase.hpp
    ${PROJECT_SOURCE_DIR}/include/States/SimulationManager/SimulationManagerGUI.hpp
    ${PROJECT_SOURCE_DIR}/include/States/SimulationManager/PlaybackManagerBase.hpp
    ${PROJECT_SOURCE_DIR}/include/States/SimulationManager/PlaybackManager.hpp
    ${PROJECT_SOURCE_DIR}/include/States/SimulationManager/PlaybackLoader.hpp
)
SET(STATES_SRC
    ${PROJECT_SOURCE_DIR}/src/States/SimulationManager/PlaybackManager.cpp
    ${PROJECT_SOURCE_DIR}/src/States/SimulationManager/SimulationManagerGUI.cpp
    ${PROJECT_SOURCE_DIR}/src/States/AppStateManager.cpp
    ${PROJECT_SOURCE_DIR}/src/States/SimulationState.cpp
    ${PROJECT_SOURCE_DIR}/src/States/PlaybackState.cpp
    ${PROJECT_SOURCE_DIR}/src/States/SimulationManager/SimulationManagerBase.cpp
    ${PROJECT_SOURCE_DIR}/src/States/SimulationManager/PlaybackManagerBase.cpp
)

SET(COMMON_INC
    ${PROJECT_SOURCE_DIR}/include/Common/TypeDefs.hpp
    ${PROJECT_SOURCE_DIR}/include/Common/OgreMeshExtraction.hpp
    ${PROJECT_SOURCE_DIR}/include/Common/AxisObject.hpp
    ${PROJECT_SOURCE_DIR}/include/Common/DynamicLines.hpp
    ${PROJECT_SOURCE_DIR}/include/Common/DynamicRenderable.hpp
    ${PROJECT_SOURCE_DIR}/include/Common/CommonFunctionsOgre.hpp
    ${PROJECT_SOURCE_DIR}/include/Common/LogDefines.hpp
    ${PROJECT_SOURCE_DIR}/include/Common/AssertionDebug.hpp
)
SET(COMMON_SRC
    ${PROJECT_SOURCE_DIR}/src/Common/OgreMeshExtraction.cpp
    ${PROJECT_SOURCE_DIR}/src/Common/AxisObject.cpp
    ${PROJECT_SOURCE_DIR}/src/Common/DynamicLines.cpp
    ${PROJECT_SOURCE_DIR}/src/Common/DynamicRenderable.cpp
    ${PROJECT_SOURCE_DIR}/src/Common/CommonFunctionsOgre.cpp
    ${PROJECT_SOURCE_DIR}/src/Common/TypeDefs.cpp
)

SET(DYNAMICS_INC
)
SET(DYNAMICS_SRC
)

SET(SYSTEMS_INC
    ${PROJECT_SOURCE_DIR}/include/Systems/SceneParserOgre.hpp
)

SET(SYSTEMS_SRC
)

# COMMON FILES INCLUDE ====================================================================
INCLUDE(IncludeModulesSimulationFramework)

INCLUDE_SIMULATION_FRAMEWORK_GUI( GLOBAL_COMMON_SRC  GLOBAL_COMMON_INC GLOBAL_COMMON_INC_DIR ${COMMONSOURCE_DIR})

# ===========================================================================================

# CUDA SOURCE INCLUDES ====================================================================
if(SIMULATIONFRAMEWORK_USE_CUDA)
MESSAGE(ERROR "CUDA does not work, write library to include!")
INCLUDE(IncludeModulesCUDA)

# Configure File for the BUILD!
string(REPLACE "." "" CUDA_VERSION_INTEGER ${CUDA_VERSION})

INCLUDE_PROX_CUDA(ProxGPU_SRC ProxGPU_INC CUDA_COMMON_INCLUDE_DIR ${CUDA_COMMON_SOURCE_DIR})
INCLUDE_VECTOR_ADD_CUDA(VectorAddGPU_SRC VectorAddGPU_INC CUDA_COMMON_INCLUDE_DIR ${CUDA_COMMON_SOURCE_DIR})
INCLUDE_MATRIX_VECTOR_MULT_CUDA(MatrixVectorMultGPU_SRC MatrixVectorMultGPU_INC CUDA_COMMON_INCLUDE_DIR ${CUDA_COMMON_SOURCE_DIR})
INCLUDE_GENERAL_EXTERN_CUDA(GeneralCommonFiles_SRC GeneralCommonFiles_INC CUDA_COMMON_INCLUDE_DIR ${CUDA_COMMON_SOURCE_DIR})


SET(CUDA_SRC
   ${ProxGPU_SRC} ${VectorAddGPU_SRC} ${MatrixVectorMultGPU_SRC} ${GeneralCommonFiles_SRC}
)
SET(CUDA_INC
   ${ProxGPU_INC} ${VectorAddGPU_INC} ${MatrixVectorMultGPU_INC} ${GeneralCommonFiles_INC}
)
else()
# Include nothing
endif()
# ===========================================================================================


# WRITE CONFIGURATION FILE
configure_file (
  ${PROJECT_SOURCE_DIR}/include/ConfigFiles/ConfigureFile.hpp.in
  ${PROJECT_BINARY_DIR}/ConfigFiles/ConfigureFile.hpp
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
)


#Set CEGUI Libraries
#SET(CEGUI_LIBS CEGUIBase CEGUIOgreRenderer)

SET(CMAKE_DEBUG_POSTFIX "_d")

#include directories
set(INCLUDE_DIRS

    ${Boost_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIR}
    ${OGRE_INCLUDE_DIRS}
    ${OIS_INCLUDE_DIRS}
    #${CEGUI_INCLUDE_DIR}
    ${STLSOFT_INCLUDE_DIR}
    ${TICPP_INCLUDE_DIRS}
    ${ASSIMP_INCLUDE_DIRS}
    ${COLLISIONLIB_INCLUDE_DIRS}


    ${PROJECT_BINARY_DIR}/ConfigFiles/
    ${PROJECT_SOURCE_DIR}/include/App
    ${PROJECT_SOURCE_DIR}/include/Common
    ${PROJECT_SOURCE_DIR}/include/Singeltons/Contexts
    ${PROJECT_SOURCE_DIR}/include/Singeltons/
    ${PROJECT_SOURCE_DIR}/include/States
    ${PROJECT_SOURCE_DIR}/include/States/SimulationManager
    ${PROJECT_SOURCE_DIR}/include/Systems

    ${GLOBAL_COMMON_INC_DIR}


    ${CUDA_COMMON_INCLUDE_DIR}
)

include_directories(${INCLUDE_DIRS})
PRINTLIST("Include directories are:" "${INCLUDE_DIRS}")



#link directories
set(LINK_DIRS
    #"${NVIDIA_SDK_DIR}/common/lib"
    #"${NVIDIA_SDK_DIR}../shared/lib"
    #"${CEGUI_LIBRARY_DIR}"
    ${Boost_LIBRARY_DIRS}
)
link_directories(${LINK_DIRS})
PRINTLIST( "Link directories are:" "${LINK_DIRS}")



set(LINK_LIBS ${OGRE_LIBRARIES} ${OIS_LIBRARIES} ${Boost_LIBRARIES} ${CUDA_cublas_LIBRARY} ${TICPP_LIBRARIES} ${COLLISIONLIB_LIBRARIES} ${ASSIMP_LIBRARIES})
PRINTLIST( "Linked libraries are:" "${LINK_LIBS}")


source_group("Source Files\\App" FILES ${APP_SRC})
source_group("Source Files\\Singeltons" FILES ${SINGELTON_SRC})
source_group("Source Files\\States" FILES ${STATES_SRC})
source_group("Source Files\\Common" FILES ${COMMON_SRC})
source_group("Source Files\\Dynamics" FILES ${DYNAMICS_SRC})
source_group("Source Files\\Dynamics\\Cuda\\Prox" FILES ${ProxGPU_SRC})
source_group("Source Files\\Dynamics\\Cuda\\VectorAdd" FILES ${VectorAddGPU_SRC})
source_group("Source Files\\Dynamics\\Cuda\\MatrixVectorMult" FILES ${MatrixVectorMultGPU_SRC})
source_group("Source Files\\Dynamics\\Cuda\\General" FILES ${GeneralCommonFiles_SRC})
source_group("Source Files\\Systems" FILES ${SYSTEMS_SRC})

source_group("Header Files\\ConfigFiles" FILES ${CONFIG_INC})
source_group("Header Files\\App" FILES ${APP_INC})
source_group("Header Files\\Singeltons" FILES ${SINGELTON_INC})
source_group("Header Files\\States" FILES ${STATES_INC})
source_group("Header Files\\Common" FILES ${COMMON_INC})
source_group("Header Files\\Dynamics" FILES ${DYNAMICS_INC})
source_group("Header Files\\Dynamics\\Cuda\\Prox" FILES ${ProxGPU_INC})
source_group("Header Files\\Dynamics\\Cuda\\VectorAdd" FILES ${VectorAddGPU_INC})
source_group("Header Files\\Dynamics\\Cuda\\MatrixVectorMult" FILES ${MatrixVectorMultGPU_INC})
source_group("Header Files\\Dynamics\\Cuda\\General" FILES ${GeneralCommonFiles_INC})
source_group("Header Files\\Systems" FILES ${SYSTEMS_INC})




message(STATUS "Add executable:...")
if(SIMULATIONFRAMEWORK_USE_CUDA)
CUDA_ADD_EXECUTABLE(${PROJECT_NAME}  ${SOURCE_FILES} ${INCLUDE_FILES} )
else()
ADD_EXECUTABLE(${PROJECT_NAME}  ${SOURCE_FILES} ${INCLUDE_FILES} )
endif()
target_link_libraries(${PROJECT_NAME} ${LINK_LIBS} )


SET(INSTALL_DIR_PROJECT ${PROJECT_BINARY_DIR}/bin/)
set_target_properties(${PROJECT_NAME} PROPERTIES
RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_BINARY_DIR}/bin/Debug
RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_BINARY_DIR}/bin/Release
)


# Install all files
if(WIN32)

#   install(TARGETS ${PROJECT_NAME}
#       RUNTIME DESTINATION ${INSTALL_DIR_PROJECT}/Debug
#       CONFIGURATIONS Debug)
#   install(TARGETS ${PROJECT_NAME}
#       RUNTIME DESTINATION ${INSTALL_DIR_PROJECT}/Release
#       CONFIGURATIONS Release)


    install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/resources/release/plugins.cfg
        ${CMAKE_CURRENT_SOURCE_DIR}/resources/install/resources.cfg
                #${CMAKE_CURRENT_SOURCE_DIR}/resources/install/SceneFile.xml
        ${CMAKE_CURRENT_SOURCE_DIR}/resources/release/ogre.cfg
        DESTINATION ${INSTALL_DIR_PROJECT}/Release
        CONFIGURATIONS Release
    )
    install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/resources/debug/plugins.cfg
        ${CMAKE_CURRENT_SOURCE_DIR}/resources/install/resources.cfg
               # ${CMAKE_CURRENT_SOURCE_DIR}/resources/install/SceneFile.xml
        ${CMAKE_CURRENT_SOURCE_DIR}/resources/debug/ogre.cfg
        DESTINATION ${INSTALL_DIR_PROJECT}/Debug
        CONFIGURATIONS Debug
    )

    install(FILES ${OGRE_BINARY_REL}
        ${OGRE_RenderSystem_Direct3D11_REL}
        ${OGRE_RenderSystem_Direct3D10_REL}
        ${OGRE_RenderSystem_Direct3D9_REL}
        ${OGRE_RenderSystem_GL_REL}
        ${OIS_BINARY_REL}
        ${OPCODE_REL}
        ${ASSIMP_REL}
        DESTINATION ${INSTALL_DIR_PROJECT}/Release
        CONFIGURATIONS Release
    )

    install(FILES ${OGRE_BINARY_DBG}
        ${OGRE_RenderSystem_Direct3D11_DBG}
        ${OGRE_RenderSystem_Direct3D10_DBG}
        ${OGRE_RenderSystem_Direct3D9_DBG}
        ${OGRE_RenderSystem_GL_DBG}
        ${OIS_BINARY_DBG}
        ${OPCODE_DBG}
        ${ASSIMP_DBG}
        DESTINATION ${INSTALL_DIR_PROJECT}/Debug
        CONFIGURATIONS Debug
    )

   # as of sdk 1.7.2 we need to copy the boost dll's as well
   # because they're not linked statically (it worked with 1.7.1 though)
   install(FILES ${Boost_DATE_TIME_LIBRARY_RELEASE}
      ${Boost_THREAD_LIBRARY_RELEASE}
      DESTINATION ${INSTALL_DIR_PROJECT}/Release
      CONFIGURATIONS Release
   )
   install(FILES ${Boost_DATE_TIME_LIBRARY_DEBUG}
      ${Boost_THREAD_LIBRARY_DEBUG}
      DESTINATION ${INSTALL_DIR_PROJECT}/Debug
      CONFIGURATIONS Debug
   )

endif(WIN32)

if(UNIX)

#   install(TARGETS ${PROJECT_NAME}
#       RUNTIME DESTINATION ${INSTALL_DIR_PROJECT}/Debug
#       CONFIGURATIONS Debug)
#   install(TARGETS ${PROJECT_NAME}
#       RUNTIME DESTINATION ${INSTALL_DIR_PROJECT}/Release
#       CONFIGURATIONS Release)


    install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/resources/release/unix/plugins.cfg
        ${CMAKE_CURRENT_SOURCE_DIR}/resources/install/resources.cfg
              #  ${CMAKE_CURRENT_SOURCE_DIR}/resources/install/SceneFile.xml
        ${CMAKE_CURRENT_SOURCE_DIR}/resources/release/unix/ogre.cfg
        DESTINATION ${INSTALL_DIR_PROJECT}/Release
        CONFIGURATIONS Release
    )
    install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/resources/debug/unix/plugins.cfg
        ${CMAKE_CURRENT_SOURCE_DIR}/resources/install/resources.cfg
              #  ${CMAKE_CURRENT_SOURCE_DIR}/resources/install/SceneFile.xml
        ${CMAKE_CURRENT_SOURCE_DIR}/resources/debug/unix/ogre.cfg
        DESTINATION ${INSTALL_DIR_PROJECT}/Debug
        CONFIGURATIONS Debug
    )

endif(UNIX)

message(STATUS "=========================================================================")
