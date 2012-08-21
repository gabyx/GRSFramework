
#ifndef AssertionDebug_H
#define AssertionDebug_H

// Add an Assertion Debuggin!

//#define NDEBUG
#include <assert.h>
#include <stdlib.h>
#include <iostream>
#include <typeinfo>


#if defined(__APPLE__) || defined(__linux__) || defined(UNIX)

#include <limits>
namespace debug {

	template<typename T>
	inline bool isInf(T const& x) {
    return x == std::numeric_limits<T>::infinity() ||  x == -std::numeric_limits<T>::infinity();
	}

	template<typename T>
	inline bool isNan(T value)
	{
	return value != value;
	}

	template<typename T>
	inline bool isNanAnyInf(T value)
	{
		return isInf(value) || isNan(value);
	}

}

#elif defined(WIN32) || defined(WIN64)
	#include <float.h>
	// use _isnan and _finite
	namespace debug {
		template<typename T>
		inline bool isNanAnyInf(T value)
		{
			return (_isnan(value) || !_finite(value));
		};
	}
#endif



#ifndef NDEBUG
	/**
	* @brief An Assert Macro to use within C++ code.
	* @param condition The condition which needs to be truem otherwise an assertion is thrown!
	* @param message The message in form of cout out expression like: ÒVariable:Ó<<i<< Òhas failedÓ
	*/
	 #define ASSERTMSG(condition , message) { if(!(condition)){ std::cerr << "ASSERT FAILED: " << #condition << " @ " <<std::endl<< message << std::endl << __FILE__ << " (" << __LINE__ << ")" << std::endl; abort();} }
    #define WARNINGMSG(condition , message) { if(!(condition)){ std::cerr << "WARNING : " << #condition << " @ " <<std::endl<< message << std::endl << __FILE__ << " (" << __LINE__ << ")" << std::endl;} }

#else
	#define ASSERTMSG(condition,message) (void)0;
   #define WARNINGMSG(condition,message) (void)0;
#endif

   #define ERRORMSG(message) { std::cerr << "ERROR :  @ " <<std::endl<< message << std::endl << __FILE__ << " (" << __LINE__ << ")" << std::endl; abort(); }

	/**
	* @brief An Assert Macro to use within C++ code. Checks if an variable is either INF or NaN.
	* @param a The variable to check for INF or NaN.
	*/
#define CHECKINFNAN(a) ASSERTMSG(! debug::isNanAnyInf(a),  "  " <<#a << "=" << a )

#endif
