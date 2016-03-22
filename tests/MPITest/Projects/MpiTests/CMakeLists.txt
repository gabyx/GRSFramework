
#The name of the project
PROJECT(MpiTests)
message(STATUS "==============================Project: ${PROJECT_NAME} ==================================")
include(PrintListMacro)

#The CMake Minimum version that is required. The FindCUDA script
#is distributed since version 2.8
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)



#include directories
set(INCLUDE_DIRS
${PROJECT_SOURCE_DIR}/inc
${COMMON_INCLUDE_DIR}
${EIGEN_INCLUDE_DIR}
${MPI_CXX_INCLUDE_PATH}
)

include_directories(${INCLUDE_DIRS})
PRINTLIST("Include Directories are:" "${INCLUDE_DIRS}")

#link directories
link_directories(${LINK_DIRS})
PRINTLIST("Link directories are:" "${LINK_DIRS}")

set(LINKED_LIBS ${MPI_CXX_LIBRARIES} ${Boost_LIBRARIES})
PRINTLIST("Linked libraries are:" "${LINKED_LIBS}")



set(SOURCE_FILES
	"src/main.cpp"
	${COMMON_FILES_SRC}
)

set(INCLUDE_FILES
	${COMMON_FILES_INC}
        "inc/MpiTests.hpp"
        "inc/SerializationTests.hpp"
)

SET(CMAKE_DEBUG_POSTFIX "D")

set_source_files_properties( INCLUDE_FILES PROPERTIES HEADER_FILE_ONLY TRUE)


#The rule to build the executable program

#make_groups_common_files()# add a mpi run test

#ADD_TEST("${PROJECT_NAME}-RUNTEST" ${MPIEXEC} ${MPIEXEC_NUMPROC_FLAG} 4 ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME})

ADD_EXECUTABLE(${PROJECT_NAME} ${SOURCE_FILES} ${INCLUDE_FILES} )

target_link_libraries(${PROJECT_NAME}  ${LINKED_LIBS})




message(STATUS "=========================================================================")
