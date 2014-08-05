
include(PrintListMacro)

PROJECT(Test)

#Set Ogre Libraries
#SET(OGRE_LIBS OgreMain OIS )

#Set CEGUI Libraries
#SET(CEGUI_LIBS CEGUIBase CEGUIOgreRenderer)


SET(CMAKE_DEBUG_POSTFIX "_d")

include_directories(
	#${NVIDIA_SDK_DIR}/common/inc
	#${NVIDIA_SDK_DIR}/../shared/inc
	${Boost_INCLUDE_DIRS}
	${EIGEN_INCLUDE_DIR}
	#${OGRE_INCLUDE_DIR}
	#${OGRE_INCLUDE_DIR}/OGRE
	#${OGRE_INCLUDE_DIR}/OIS
	#${CEGUI_INCLUDE_DIR}
	#${STLSOFT_INCLUDE_DIRS}
	"${PROJECT_SOURCE_DIR}/include"
)


#link directories
link_directories(
	#${NVIDIA_SDK_DIR}/common/lib
	#${NVIDIA_SDK_DIR}../shared/lib
	#${OGRE_LIBRARY_DIR}/lib/
	#${CEGUI_LIBRARY_DIR}
	${Boost_LIBRARY_DIRS}
)

#link directories
set(LINK_DIRS
${Boost_LIBRARY_DIRS}
)
link_directories(${LINK_DIRS})
PRINTLIST("Link directories are:" "${LINK_DIRS}")
set(LINKED_LIBS ${Boost_LIBRARIES})
PRINTLIST("Linked libraries are:" "${LINKED_LIBS}")



#Set Source Code 

SET(SOURCE_FILES 
	${PROJECT_SOURCE_DIR}/src/main.cpp
)

SET(INCLUDE_FILES 
)

IF(MSVC)

source_group("Common Source" FILES ${COMMON_FILES_SRC})
source_group("Common Header" FILES ${COMMON_FILES_INC})
CUDA_ADD_EXECUTABLE(Test ${SOURCE_FILES} ${INCLUDE_FILES} ${COMMON_FILES_SRC} ${COMMON_FILES_INC})
ELSE(MSVC)
CUDA_ADD_EXECUTABLE(Test ${SOURCE_FILES} ${INCLUDE_FILES} ${COMMON_FILES_SRC} ${COMMON_FILES_INC})
ENDIF(MSVC)
target_link_libraries(Test ${LINKED_LIBS} )

