

PROJECT(Converter)

message(STATUS "==============================Project: ${PROJECT_NAME} ==================================")

include(PrintListMacro)


#Set Source Code (its in the GeneralRigidBodySimulation Folder)
SET(SOURCE_DIR "${PROJECT_SOURCE_DIR}/../GeneralRigidBodySimulation/")

SET(CONFIG_INC ${PROJECT_BINARY_DIR}/ConfigFiles/ConfigureFile.hpp)






SET( APP_INC
)
SET( APP_SRC
    ${PROJECT_SOURCE_DIR}/src/App/main.cpp
)

SET(SOURCE_DIR "${SOURCE_DIR}/../GeneralRigidBodySimulation/")


SET(COMMON_INC

    ${SOURCE_DIR}/include/Common/AssertionDebug.hpp
    ${PROJECT_SOURCE_DIR}/include/Common/TypeDefs.hpp
    ${PROJECT_SOURCE_DIR}/include/Common/LogDefines.hpp

    ${PROJECT_SOURCE_DIR}/include/Common/ApplicationCLOptionsConverter.hpp

    ${PROJECT_SOURCE_DIR}/include/Converters/SimFileJoiner.hpp
    ${PROJECT_SOURCE_DIR}/include/Converters/SimFileResampler.hpp

    ${PROJECT_SOURCE_DIR}/include/Converters/RenderScriptConverter.hpp
    
    ${PROJECT_SOURCE_DIR}/include/Logic/LogicTypes.hpp

    
    ${PROJECT_SOURCE_DIR}/include/General/RenderData.hpp
    ${PROJECT_SOURCE_DIR}/include/General/RenderScriptParserGenerators.hpp
    
    ${PROJECT_SOURCE_DIR}/include/General/RenderScriptParser.hpp
    ${PROJECT_SOURCE_DIR}/include/General/RenderScriptParserBaseTraits.hpp
    ${PROJECT_SOURCE_DIR}/include/General/RenderScriptParserModules.hpp
    
    ${PROJECT_SOURCE_DIR}/include/General/RenderScriptGenerator.hpp
    ${PROJECT_SOURCE_DIR}/include/General/RenderScriptGeneratorLogic.hpp
    ${PROJECT_SOURCE_DIR}/include/General/RendermanGeometryWriter.hpp
    

        
    
    ${PROJECT_SOURCE_DIR}/include/General/RenderMaterial.hpp
)
SET(COMMON_SRC
    ${SOURCE_DIR}/src/Common/TypeDefs.cpp
    ${PROJECT_SOURCE_DIR}/src/Converters/RenderScriptConverter.cpp
    ${PROJECT_SOURCE_DIR}/src/General/RenderData.cpp
    ${PROJECT_SOURCE_DIR}/src/General/RenderScriptGenerator.cpp
)


# COMMON FILES INCLUDE ====================================================================
INCLUDE(IncludeModulesSimulationFramework)

INCLUDE_SIMULATION_FRAMEWORK_CONVERTER( GLOBAL_COMMON_SRC  GLOBAL_COMMON_INC GLOBAL_COMMON_INC_DIR ${COMMONSOURCE_DIR})

# ===========================================================================================
# WRITE CONFIGURATION FILE
configure_file (
  ${SOURCE_DIR}/include/ConfigFiles/ConfigureFile.hpp.in
  ${PROJECT_BINARY_DIR}/ConfigFiles/ConfigureFile.hpp
)
#=========================


SET(SOURCE_FILES
    ${APP_SRC}
    ${COMMON_SRC}
    ${DYNAMICS_SRC}
    ${CUDA_SRC}
    ${GLOBAL_COMMON_SRC}
)

SET(INCLUDE_FILES
    ${APP_INC}
    ${COMMON_INC}
    ${DYNAMICS_INC}
    ${CUDA_INC}
    ${GLOBAL_COMMON_INC}
)


SET(CMAKE_DEBUG_POSTFIX "_d")

#include directories
set(INCLUDE_DIRS
    ${Boost_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIR}

    ${PROJECT_BINARY_DIR}/ConfigFiles/

	${COMMONSOURCE_DIR}/external/getoptpp/
    ${COMMONSOURCE_DIR}/external/getoptpp/src/

    ${PROJECT_SOURCE_DIR}/include/App
	${PROJECT_SOURCE_DIR}/include/Common
    ${PROJECT_SOURCE_DIR}/include/Converters
    ${PROJECT_SOURCE_DIR}/include/General/
    ${PROJECT_SOURCE_DIR}/include/Logic/


	${SOURCE_DIR}/include/Common

    ${GLOBAL_COMMON_INC_DIR}

)

include_directories(${INCLUDE_DIRS})
PRINTLIST("Include directories are:" "${INCLUDE_DIRS}")



#link directories
set(LINK_DIRS
    ${Boost_LIBRARY_DIRS}
)
link_directories(${LINK_DIRS})
PRINTLIST( "Link directories are:" "${LINK_DIRS}")



set(LINK_LIBS ${Boost_LIBRARIES})
PRINTLIST( "Linked libraries are:" "${LINK_LIBS}")


source_group("Source Files\\App" FILES ${APP_SRC})
source_group("Source Files\\Common" FILES ${COMMON_SRC})
source_group("Source Files\\Dynamics" FILES ${DYNAMICS_SRC})

source_group("Header Files\\App" FILES ${APP_INC})
source_group("Header Files\\Common" FILES ${COMMON_INC})
source_group("Header Files\\Dynamics" FILES ${DYNAMICS_INC})


message(STATUS "Add executable:...")
ADD_EXECUTABLE(${PROJECT_NAME}  ${SOURCE_FILES} ${INCLUDE_FILES})
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


    #install(FILES
                #${CMAKE_CURRENT_SOURCE_DIR}/resources/install/SceneFile.xml
        #DESTINATION ${INSTALL_DIR_PROJECT}/Release
        #CONFIGURATIONS Release
    #)
    #install(FILES
                #${CMAKE_CURRENT_SOURCE_DIR}/resources/install/SceneFile.xml
        #DESTINATION ${INSTALL_DIR_PROJECT}/Debug
        #CONFIGURATIONS Debug
    #)

    #install(FILES
        #${COLLISIONLIB_REL}
        #${ASSIMP_REL}
        #DESTINATION ${INSTALL_DIR_PROJECT}/Release
        #CONFIGURATIONS Release
    #)

    #install(FILES
        #${COLLISIONLIB_DBG}
        #${ASSIMP_DBG}
        #DESTINATION ${INSTALL_DIR_PROJECT}/Debug
        #CONFIGURATIONS Debug
    #)

   # as of sdk 1.7.2 we need to copy the boost dll's as well
   # because they're not linked statically (it worked with 1.7.1 though)
   #install(FILES ${Boost_DATE_TIME_LIBRARY_RELEASE}
      #${Boost_THREAD_LIBRARY_RELEASE}
      #DESTINATION ${INSTALL_DIR_PROJECT}/Release
      #CONFIGURATIONS Release
   #)
   #install(FILES ${Boost_DATE_TIME_LIBRARY_DEBUG}
      #${Boost_THREAD_LIBRARY_DEBUG}
      #DESTINATION ${INSTALL_DIR_PROJECT}/Debug
      #CONFIGURATIONS Debug
   #)

endif(WIN32)

if(UNIX)

#   install(TARGETS ${PROJECT_NAME}
#       RUNTIME DESTINATION ${INSTALL_DIR_PROJECT}/Debug
#       CONFIGURATIONS Debug)
#   install(TARGETS ${PROJECT_NAME}
#       RUNTIME DESTINATION ${INSTALL_DIR_PROJECT}/Release
#       CONFIGURATIONS Release)


    #install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/resources/install/SceneFile.xml
    #    DESTINATION ${INSTALL_DIR_PROJECT}/Release
    #    CONFIGURATIONS Release
   # )
    #install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/resources/install/SceneFile.xml
    #    DESTINATION ${INSTALL_DIR_PROJECT}/Debug
    #    CONFIGURATIONS Debug
    #)

endif(UNIX)

message(STATUS "=========================================================================")
