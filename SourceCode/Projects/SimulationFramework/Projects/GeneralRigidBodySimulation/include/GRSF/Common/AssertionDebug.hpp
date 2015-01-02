
#ifndef GRSF_Common_AssertionDebug_hpp
#define GRSF_Common_AssertionDebug_hpp

// Add an Assertion Debuggin!

//#define NDEBUG
#include <stdlib.h>
#include <iostream>
#include <typeinfo>

#include "GRSF/Common/Exception.hpp"

#ifndef NDEBUG
// Debug!
	/**
	* @brief An Assert Macro to use within C++ code.
	* @param condition The condition which needs to be truem otherwise an assertion is thrown!
	* @param message The message in form of cout out expression like: "Variable" << i<< "has failed"
	*/
    #define ASSERTMSG(condition , message) { if(!(condition)){ ERRORMSG(message) } }
    #define WARNINGMSG(condition , message) { if(!(condition)){ ERRORMSG(message) } }
#else
   #define ASSERTMSG(condition,message)
   #define WARNINGMSG(condition,message)
#endif

   #define ERRORMSG( message ) THROWEXCEPTION( message )

#endif
