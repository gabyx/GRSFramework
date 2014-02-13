
#ifndef AssertionDebug_H
#define AssertionDebug_H

// Add an Assertion Debuggin!

//#define NDEBUG
#include <assert.h>
#include <stdlib.h>
#include <iostream>
#include <typeinfo>


#ifndef NDEBUG
// Debug!
	/**
	* @brief An Assert Macro to use within C++ code.
	* @param condition The condition which needs to be truem otherwise an assertion is thrown!
	* @param message The message in form of cout out expression like: "Variable" << i<< "has failed"
	*/
    #define ASSERTMSG(condition , message) { if(!(condition)){ std::cerr << "ASSERT FAILED: " << #condition << " @ " <<std::endl<< message << std::endl << __FILE__ << " (" << __LINE__ << ")" << std::endl; abort();} }
    #define WARNINGMSG(condition , message) { if(!(condition)){ std::cerr << "WARNING : " << #condition << " @ " <<std::endl<< message << std::endl << __FILE__ << " (" << __LINE__ << ")" << std::endl;} }

#else
//   #define ASSERTMSG(condition,message) (void)0;
//   #define WARNINGMSG(condition,message) (void)0;
   #define ASSERTMSG(condition , message) { if(!(condition)){ std::cerr << "ASSERT FAILED: " << #condition << " @ " <<std::endl<< message << std::endl << __FILE__ << " (" << __LINE__ << ")" << std::endl; abort();} }
   #define WARNINGMSG(condition , message) { if(!(condition)){ std::cerr << "WARNING : " << #condition << " @ " <<std::endl<< message << std::endl << __FILE__ << " (" << __LINE__ << ")" << std::endl;} }

#endif

   #define ERRORMSG(message) { std::cerr << "ERROR :  @ " <<std::endl<< message << std::endl << __FILE__ << " (" << __LINE__ << ")" << std::endl; abort(); }

#endif
