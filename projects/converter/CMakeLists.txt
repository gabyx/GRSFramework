

PROJECT(GRSFConverter)


#Define H5 ===================================================================================================================================================
find_package(HDF5 COMPONENTS CXX HL REQUIRED)
# ===============================================================================================================================================================



message(STATUS "==============================Project: ${PROJECT_NAME} ==================================")

include(PrintListMacro)



SET(SOURCE_DIR "${PROJECT_SOURCE_DIR}/../granularRigidBodySimulationGUI/")
SET(CONFIG_INC ${PROJECT_BINARY_DIR}/configFiles/ConfigureFile.hpp)


SET( APP_INC
)
SET( APP_SRC
    ${PROJECT_SOURCE_DIR}/src/GRSF/app/main.cpp
)


SET(COMMON_INC

    ${SOURCE_DIR}/include/GRSF/common/Asserts.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/common/TypeDefs.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/common/LogDefines.hpp

    ${PROJECT_SOURCE_DIR}/include/GRSF/common/ApplicationCLOptionsConverter.hpp

    ${PROJECT_SOURCE_DIR}/include/GRSF/converters/simJoiner/SimFileJoiner.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/converters/simResampler/SimFileResampler.hpp
    
    ${PROJECT_SOURCE_DIR}/include/GRSF/logic/LogicTypes.hpp
    
    ${PROJECT_SOURCE_DIR}/include/GRSF/converters/SimFileConverter.hpp
    
    
    ${PROJECT_SOURCE_DIR}/include/GRSF/general/LogicParser.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/general/LogicParserTraits.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/general/LogicParserTraitsMacro.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/general/LogicParserBaseTraits.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/general/LogicParserBaseTraitsMacro.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/general/LogicParserModules.hpp
    
    
    # Gridder
    ${PROJECT_SOURCE_DIR}/include/GRSF/converters/gridder/GridderConverter.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/converters/gridder/GridderData.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/converters/gridder/GridderLogicParser.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/converters/gridder/GridderLogicParserGenerators.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/converters/gridder/GridderLogicParserTraits.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/converters/gridder/GridderLogicParserTraitsMacro.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/converters/gridder/GridExtractor.hpp

    # Analyzer
    ${PROJECT_SOURCE_DIR}/include/GRSF/converters/analyzer/AnalyzerConverter.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/general/SimFileExecutionGraph.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/general/SimFileExecutionGraphNodes.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/converters/analyzer/AnalyzerLogicParser.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/converters/analyzer/AnalyzerLogicParserGenerators.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/converters/analyzer/AnalyzerLogicParserTraits.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/converters/analyzer/AnalyzerLogicParserTraitsMacro.hpp
    
    #Render
    ${PROJECT_SOURCE_DIR}/include/GRSF/converters/renderer/RenderConverter.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/converters/renderer/RenderData.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/converters/renderer/RenderExecutionGraph.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/converters/renderer/RenderExecutionGraphNodes.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/converters/renderer/RenderLogicParser.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/converters/renderer/RenderLogicParserTraits.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/converters/renderer/RenderLogicParserTraitsMacro.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/converters/renderer/RenderLogicParserGenerators.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/converters/renderer/RenderLogicParserModules.hpp

    ${PROJECT_SOURCE_DIR}/include/GRSF/converters/renderer/RendermanGeometryWriter.hpp
    ${PROJECT_SOURCE_DIR}/include/GRSF/converters/renderer/RenderMaterial.hpp
)
SET(COMMON_SRC
    ${SOURCE_DIR}/src/GRSF/common/TypeDefs.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/converters/gridder/GridderConverter.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/converters/gridder/GridExtractor.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/converters/renderer/RenderConverter.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/converters/analyzer/AnalyzerConverter.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/converters/renderer/RenderData.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/converters/renderer/RenderExecutionGraph.cpp
    ${PROJECT_SOURCE_DIR}/src/GRSF/general/SimFileExecutionGraph.cpp

)


# COMMON FILES INCLUDE ====================================================================
INCLUDE(IncludeModulesGRSFramework)

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
SET(CONFIG_INC ${PROJECT_BINARY_DIR}/include/GRSF/configFiles/ConfigureFile.hpp)
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
    RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_BINARY_DIR}/bin/debug
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_BINARY_DIR}/bin/release
)

add_dependencies(${PROJECT_NAME}  ${ApproxMVBB_DEPENDING_TARGETS})


message(STATUS "=========================================================================")
