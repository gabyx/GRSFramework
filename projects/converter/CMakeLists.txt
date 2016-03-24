

PROJECT(GRSFConverter)


#Define H5 ===================================================================================================================================================
find_package(HDF5 COMPONENTS CXX HL REQUIRED)
# ===============================================================================================================================================================



message(STATUS "==============================Project: ${PROJECT_NAME} ==================================")

include(PrintListMacro)


#Set Source Code (its in the GeneralRigidBodySimulation Folder)
SET(SOURCE_DIR "${PROJECT_SOURCE_DIR}/../granularRigidBodySimulation/")

SET(CONFIG_INC ${PROJECT_BINARY_DIR}/ConfigFiles/ConfigureFile.hpp)


SET( APP_INC
)
SET( APP_SRC
    ${PROJECT_SOURCE_DIR}/src/GRSF/App/main.cpp
)

SET(SOURCE_DIR "${SOURCE_DIR}/../granularRigidBodySimulation/")


SET(COMMON_INC

    ${SOURCE_DIR}/include/GRSF/Common/AssertionDebug.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Common/TypeDefs.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Common/LogDefines.hpp

    ${PROJECT_SOURCE_DIR}/include/GRSF/Common/ApplicationCLOptionsConverter.hpp

    ${PROJECT_SOURCE_DIR}/include/GRSF/Converters/SimJoiner/SimFileJoiner.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Converters/SimResampler/SimFileResampler.hpp
    
    ${PROJECT_SOURCE_DIR}/include/GRSF/Logic/LogicTypes.hpp
    
    ${PROJECT_SOURCE_DIR}/include/GRSF/Converters/SimFileConverter.hpp
    
    
    ${PROJECT_SOURCE_DIR}/include/GRSF/General/LogicParser.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/General/LogicParserTraits.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/General/LogicParserTraitsMacro.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/General/LogicParserBaseTraits.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/General/LogicParserBaseTraitsMacro.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/General/LogicParserModules.hpp
    
    
    # Gridder
    ${PROJECT_SOURCE_DIR}/include/GRSF/Converters/Gridder/GridderConverter.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Converters/Gridder/GridderData.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Converters/Gridder/GridderLogicParser.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Converters/Gridder/GridderLogicParserGenerators.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Converters/Gridder/GridderLogicParserTraits.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Converters/Gridder/GridderLogicParserTraitsMacro.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Converters/Gridder/GridExtractor.hpp

    # Analyzer
    ${PROJECT_SOURCE_DIR}/include/GRSF/Converters/Analyzer/AnalyzerConverter.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/General/SimFileExecutionGraph.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/General/SimFileExecutionGraphNodes.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Converters/Analyzer/AnalyzerLogicParser.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Converters/Analyzer/AnalyzerLogicParserGenerators.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Converters/Analyzer/AnalyzerLogicParserTraits.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Converters/Analyzer/AnalyzerLogicParserTraitsMacro.hpp
    
    #Render
    ${PROJECT_SOURCE_DIR}/include/GRSF/Converters/Renderer/RenderConverter.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Converters/Renderer/RenderData.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Converters/Renderer/RenderExecutionGraph.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Converters/Renderer/RenderExecutionGraphNodes.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Converters/Renderer/RenderLogicParser.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Converters/Renderer/RenderLogicParserTraits.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Converters/Renderer/RenderLogicParserTraitsMacro.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Converters/Renderer/RenderLogicParserGenerators.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Converters/Renderer/RenderLogicParserModules.hpp

    ${PROJECT_SOURCE_DIR}/include/GRSF/Converters/Renderer/RendermanGeometryWriter.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/Converters/Renderer/RenderMaterial.hpp
)
SET(COMMON_SRC
    ${SOURCE_DIR}/src/GRSF/Common/TypeDefs.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/Converters/Gridder/GridderConverter.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/Converters/Gridder/GridExtractor.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/Converters/Renderer/RenderConverter.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/Converters/Analyzer/AnalyzerConverter.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/Converters/Renderer/RenderData.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/Converters/Renderer/RenderExecutionGraph.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/General/SimFileExecutionGraph.cpp

)


# COMMON FILES INCLUDE ====================================================================
INCLUDE(IncludeModulesSimulationFramework)

INCLUDE_SIMULATION_FRAMEWORK_CONVERTER( GLOBAL_COMMON_SRC  GLOBAL_COMMON_INC GLOBAL_COMMON_INC_DIR ${GRSF_COMMON_SOURCE_DIR})

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
  ${SOURCE_DIR}/include/GRSF/configFiles/ConfigureFile.hpp.in
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
    
    ${HDF5_INCLUDE_DIRS}

    ${GRSF_DEP_INCLUDE_DIRS}
    
    ${PROJECT_BINARY_DIR}/include/
    ${PROJECT_SOURCE_DIR}/include/
    ${SOURCE_DIR}/include/
    
    ${GLOBAL_COMMON_INC_DIR}
    ${ApproxMVBB_INC_DIRS}

)

include_directories(${INCLUDE_DIRS})
PRINTLIST("Include directories are:" "${INCLUDE_DIRS}")

#link directories
set(LINK_DIRS
    ${GRSF_DEP_LIBRARY_DIRS}
)
link_directories(${LINK_DIRS})
PRINTLIST( "Link directories are:" "${LINK_DIRS}")


set(LINK_LIBS ${GRSF_DEP_LIBRARIES} ${HDF5_LIBRARIES})
PRINTLIST( "Linked libraries are:" "${LINK_LIBS}")


message(STATUS "Add executable:  ${PPROJECT_NAME}")
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
