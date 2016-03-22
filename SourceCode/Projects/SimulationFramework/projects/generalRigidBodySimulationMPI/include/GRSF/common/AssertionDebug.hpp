
#ifndef GRSF_Common_AssertionDebug_hpp
#define GRSF_Common_AssertionDebug_hpp

// Add an Assertion Debuggin!

//#define NDEBUG
#include <assert.h>
#include <stdlib.h>
#include <iostream>
#include <typeinfo>

#include <mpi.h>

#include "GRSF/Common/Exception.hpp"
#include "GRSF/Common/LogDefines.hpp"

// This is the MPI version for our Debugging shit =)

#ifndef NDEBUG
	/**
	* @brief An Assert Macro to use within C++ MPI code.
	* @param condition The condition which needs to be truem otherwise an assertion is thrown!
	* @param message The message in form of cout out expression like: "Variable" << i<< "has failed"
	*/

	#define LOGASSERTMSG( condition , log , message ) { \
        if( !( condition ) ){                              \
            LOG( log , message );                    \
            ERRORMSG( message );                   \
        }\
    }

	#define ASSERTMSG(condition , message) { \
        if( !( condition ) ){                      \
            ERRORMSG( message );                   \
        }\
    }

    #define WARNINGMSG(condition , message) { if(!(condition)){ std::cerr << "WARNING: " << #condition << " : " <<std::endl<< message << std::endl << " @ " << __FILE__ << " (" << __LINE__ << ")" << std::endl;} }

#else
   //#define ASSERTMSG(condition,message);
   //#define WARNINGMSG(condition,message);
   //#define LOGASSERTMSG( _log_ , _assert_ , _statement_ );

   #define LOGASSERTMSG( condition , log , message ) { \
        if( !( condition ) ){                              \
            LOG( log , message );                    \
            ERRORMSG( message );                   \
        }\
    }

	#define ASSERTMSG(condition , message) { \
        if( !( condition ) ){                      \
            ERRORMSG( message );                   \
        }\
    }

    #define WARNINGMSG(condition , message) { if(!(condition)){ std::cerr << "WARNING: " << #condition << " : " <<std::endl<< message << std::endl << " @ " << __FILE__ << " (" << __LINE__ << ")" << std::endl;} }



#endif


/**
	* @brief An Error Macro to use within C++ for MPI code.
    * Writes in a global file!
	*/
#define ERRORMSG( _message_ ) THROWEXCEPTION( _message_ );

#define ERRORMSG2( _message1_ , _message2_ ) ERRORMSG( _message1_ << _message2_ )


#ifndef NDEBUG
	/**
	* @brief An Assert Macro for MPI routines to use within C++ MPI code.
	 * Writes in a global file!
	*/
    #define ASSERTMPIERROR( error_code , message ) { \
        if(error_code != MPI_SUCCESS){  \
            char * string = nullptr; \
            int length; \
            MPI_Error_string( error_code , string, &length ); \
            ERRORMSG2( string , message ); \
        } \
    }

#else
	//#define ASSERTMPIERROR( error_code , message );

    #define ASSERTMPIERROR( error_code , message ) { \
        if(error_code != MPI_SUCCESS){  \
            char * string = nullptr; \
            int length; \
            MPI_Error_string( error_code , string, &length ); \
            ERRORMSG2( string , message ); \
        } \
    }

#endif





#endif
