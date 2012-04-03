# Find OGRE library
#
# OGRE_INCLUDE_DIR      where to find the include files
# OGRE_LIBRARY_DIR      where to find the libraries
# OGRE_LIBRARIES        list of libraries to link
# OGRE_FOUND            true if OGRE was found


STRING(COMPARE EQUAL ${MSVC_VERSION} "1500" MSVC9)
STRING(COMPARE EQUAL ${MSVC_VERSION} "1600" MSVC10)

if(MSVC9)
	SET(MSVC_FOLDERNAME "msvc9")
elseif(MSVC10)
	SET(MSVC_FOLDERNAME "msvc10")
endif(MSVC9)


SET( OGRE_LIBRARY_DIR C:/Develop/OGRE/x64/${MSVC_FOLDERNAME}/lib CACHE PATH "Alternative library directory" )
SET( OGRE_INCLUDE_DIR C:/Develop/OGRE/x64/${MSVC_FOLDERNAME}/include  CACHE PATH "Alternative include directory" )
#MARK_AS_ADVANCED( OGRE_LIBRARY_DIR OGRE_INCLUDE_DIR )

FIND_LIBRARY( OGRE_LIBRARY_DIR OgreMain PATHS ${OGRE_LIBRARY_DIR} )
message(STATUS ${OGRE_LIBRARY_DIR})
FIND_PATH( OGRE_INCLUDE_DIR OGRE/Ogre.h PATHS ${OGRE_INCLUDE_DIR} )
message(STATUS ${OGRE_INCLUDE_DIR})

GET_FILENAME_COMPONENT( OGRE_LIBRARY_DIR ${OGRE_LIBRARY_DIR} PATH )

IF( OGRE_INCLUDE_DIR AND OGRE_LIBRARY_DIR )
    SET( OGRE_LIBRARIES OgreMain )
    SET( OGRE_FOUND TRUE )
ELSE( OGRE_INCLUDE_DIR AND OGRE_LIBRARY_DIR )
    SET( OGRE_FOUND FALSE )
ENDIF( OGRE_INCLUDE_DIR AND OGRE_LIBRARY_DIR )