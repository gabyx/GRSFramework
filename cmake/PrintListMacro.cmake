#Print macro for a list
macro(PRINTLIST MYCOMMENT MYLIST)
message(STATUS "${MYCOMMENT}")
foreach(dir ${MYLIST})
	message(STATUS "      ${dir}")
endforeach(dir)
endmacro(PRINTLIST)




