

PROJECT(Converter)

message(STATUS "==============================Project: ${PROJECT_NAME} ==================================")

include(PrintListMacro)


#Set Source Code (its in the GeneralRigidBodySimulation Folder)
SET(SOURCE_DIR "${PROJECT_SOURCE_DIR}/../GeneralRigidBodySimulation/")

SET(CONFIG_INC ${PROJECT_BINARY_DIR}/ConfigFiles/ConfigureFile.hpp)






SET( APP_INC
)
SET( APP_SRC
    ${PROJECT_SOURCE_DIR}/src/GRSF/App/main.cpp
)

SET(SOURCE_DIR "${SOURCE_DIR}/../GeneralRigidBodySimulation/")


SET(COMMON_INC

    ${SOURCE_DIR}/include/GRSF/Common/AssertionDebug.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Common/TypeDefs.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Common/LogDefines.hpp

    ${PROJECT_SOURCE_DIR}/include/GRSF/Common/ApplicationCLOptionsConverter.hpp

    ${PROJECT_SOURCE_DIR}/include/GRSF/Converters/SimFileJoiner.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Converters/SimFileResampler.hpp

    ${PROJECT_SOURCE_DIR}/include/GRSF/Converters/RenderScriptConverter.hpp
    
    ${PROJECT_SOURCE_DIR}/include/GRSF/Logic/LogicTypes.hpp

    
    ${PROJECT_SOURCE_DIR}/include/GRSF/General/RenderData.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/General/RenderScriptParserGenerators.hpp
    
    ${PROJECT_SOURCE_DIR}/include/GRSF/General/RenderScriptParser.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/General/RenderScriptParserBaseTraits.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/General/RenderScriptParserModules.hpp
    
    ${PROJECT_SOURCE_DIR}/include/GRSF/General/RenderScriptGenerator.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/General/RenderScriptGeneratorLogic.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/General/RendermanGeometryWriter.hpp
    

        
    
    ${PROJECT_SOURCE_DIR}/include/GRSF/General/RenderMaterial.hpp
)
SET(COMMON_SRC
    ${SOURCE_DIR}/src/GRSF/Common/TypeDefs.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/Converters/RenderScriptConverter.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/General/RenderData.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/General/RenderScriptGenerator.cpp
)


# COMMON FILES INCLUDE ====================================================================
INCLUDE(IncludeModulesSimulationFramework)

INCLUDE_SIMULATION_FRAMEWORK_CONVERTER( GLOBAL_COMMON_SRC  GLOBAL_COMMON_INC GLOBAL_COMMON_INC_DIR ${COMMONSOURCE_DIR})

#Configure ApproxMVBB =======================================================================
include(${ApproxMVBB_CMAKE_DIR}/DefineApproxMVBBSources.cmake)
# Define all MVBB Source files
INCLUDE_ALL_ApproxMVBB_SOURCE(  ApproxMVBB_SRC 
                                ApproxMVBB_INC 
                                ApproxMVBB_INC_DIRS 
                                ApproxMVBB_DEPENDING_TARGETS
                                ${ApproxMVBB_ROOT_DIR} ${PROJECT_BINARY_DIR} )
# ===========================================================================================


# ===========================================================================================
# WRITE CONFIGURATION FILE
SET(CONFIG_INC ${PROJECT_BINARY_DIR}/include/GRSF/ConfigFiles/ConfigureFile.hpp)
configure_file (
  ${SOURCE_DIR}/include/GRSF/ConfigFiles/ConfigureFile.hpp.in
  ${CONFIG_INC}
)
#=========================


SET(SOURCE_FILES
    ${APP_SRC}
    ${COMMON_SRC}
    ${DYNAMICS_SRC}
    ${CUDA_SRC}
    ${GLOBAL_COMMON_SRC}
    
    ${ApproxMVBB_SRC}
)

SET(INCLUDE_FILES
    ${APP_INC}
    ${COMMON_INC}
    ${DYNAMICS_INC}
    ${CUDA_INC}
    ${GLOBAL_COMMON_INC}
    
    ${ApproxMVBB_INC}
)


SET(CMAKE_DEBUG_POSTFIX "_d")

#include directories
set(INCLUDE_DIRS
    ${Boost_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIR}

    ${PROJECT_BINARY_DIR}/include/
    ${PROJECT_SOURCE_DIR}/include/
    ${SOURCE_DIR}/include/
    
    ${GLOBAL_COMMON_INC_DIR}
    
    ${ApproxMVBB_INC_DIRS}

    ${GLOBAL_COMMON_INC_DIR}

)

include_directories(${INCLUDE_DIRS})
#PRINTLIST("Include directories are:" "${INCLUDE_DIRS}")



#link directories
set(LINK_DIRS
    ${Boost_LIBRARY_DIRS}
)
link_directories(${LINK_DIRS})
#PRINTLIST( "Link directories are:" "${LINK_DIRS}")


set(LINK_LIBS ${Boost_LIBRARIES})
#PRINTLIST( "Linked libraries are:" "${LINK_LIBS}")


message(STATUS "Add executable:...")
ADD_EXECUTABLE(${PROJECT_NAME}  ${SOURCE_FILES} ${INCLUDE_FILES})
target_link_libraries(${PROJECT_NAME} ${LINK_LIBS} )


SET(INSTALL_DIR_PROJECT ${PROJECT_BINARY_DIR}/bin/)
set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin/
    RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_BINARY_DIR}/bin/Debug
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_BINARY_DIR}/bin/Release
)

add_dependencies(${PROJECT_NAME}  ${ApproxMVBB_DEPENDING_TARGETS})


message(STATUS "=========================================================================")
