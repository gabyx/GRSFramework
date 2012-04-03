
file(
	GLOB_RECURSE
	files_inc
	*.hpp
	*.h
)

file(
	GLOB_RECURSE
	files_src
	*.cpp
)

SET(COMMON_FILES_INC ${COMMON_FILES_INC} ${files_inc} PARENT_SCOPE)
SET(COMMON_FILES_SRC ${COMMON_FILES_SRC} ${files_src} PARENT_SCOPE)