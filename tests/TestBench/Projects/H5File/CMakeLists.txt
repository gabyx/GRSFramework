
PROJECT(H5File_TESTS)


#Define H5 ===================================================================================================================================================
find_package(HDF5 COMPONENTS CXX HL REQUIRED)
# ===============================================================================================================================================================



message(STATUS "==============================Project: ${PROJECT_NAME} ==================================")

include(PrintListMacro)


SET(SOURCE_FILES
    ${PROJECT_SOURCE_DIR}/src/main.cpp
)

SET(INCLUDE_FILES
    
)

SET(CMAKE_DEBUG_POSTFIX "_d")

#include directories
set(INCLUDE_DIRS

    ${Boost_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIR}
    ${EIGEN_INCLUDE_DIR}/unsupported
    ${HDF5_INCLUDE_DIRS}
    
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_BINARY_DIR}/include/
    ${PROJECT_SOURCE_DIR}/include/Common # Needs to be previous to {SOURCE_DIR}/include/Common (because there is another LogDefines.hpp

)

include_directories(${INCLUDE_DIRS})
PRINTLIST("Include directories are:" "${INCLUDE_DIRS}")


#link directories
set(LINK_DIRS
    ${Boost_LIBRARY_DIRS}
)
link_directories(${LINK_DIRS})
PRINTLIST( "Link directories are:" "${LINK_DIRS}")


set(LINK_LIBS ${Boost_LIBRARIES} ${HDF5_LIBRARIES})
PRINTLIST( "Linked libraries are:" "${LINK_LIBS}")


message(STATUS "Add executable:...")
if(SIMULATIONFRAMEWORK_USE_CUDA)
CUDA_ADD_EXECUTABLE(${PROJECT_NAME}  ${SOURCE_FILES} ${INCLUDE_FILES})
else()
ADD_EXECUTABLE(${PROJECT_NAME}  ${SOURCE_FILES} ${INCLUDE_FILES} )
endif()
target_link_libraries(${PROJECT_NAME} ${LINK_LIBS} )


message(STATUS "=========================================================================")
