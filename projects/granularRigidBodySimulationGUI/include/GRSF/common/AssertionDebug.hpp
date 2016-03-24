#ifndef GRSF_common_AssertionDebug_hpp
#define GRSF_common_AssertionDebug_hpp

// Add an Assertion Debuggin!

//#define NDEBUG
#include <stdlib.h>
#include <iostream>
#include <typeinfo>

#include "GRSF/common/Exception.hpp"

#ifndef NDEBUG
// Debug!
/**
* @brief An Assert Macro to use within C++ code.
* @param condition The condition which needs to be truem otherwise an assertion is thrown!
* @param message The message in form of cout out expression like: "Variable" << i<< "has failed"
*/
    #define ASSERTMSG(condition , message) { if(!(condition)){ ERRORMSG(message) } }
#else
    #define ASSERTMSG(condition,message)

#endif
    #define WARNINGMSG(condition , message) { if(!(condition)){ std::cerr << "WARNING @ " << __FILE__ << " (" << __LINE__ << ") :" << message << std::endl;  } }
    #define ERRORMSG( message ) THROWEXCEPTION( message )

#endif
