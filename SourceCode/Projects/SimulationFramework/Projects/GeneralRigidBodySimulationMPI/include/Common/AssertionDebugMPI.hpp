
#ifndef AssertionDebugMPI_H
#define AssertionDebugMPI_H

// Add an Assertion Debuggin!

//#define NDEBUG
#include <assert.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <typeinfo>

#include <boost/filesystem.hpp>

#include <mpi.h>



#define ERRORMSGMPI( _message_ ) { \
std::cerr << "ERROR :  @ " << std::endl << _message_ ; \
std::cerr << std::endl << "in File: " << __FILE__ << "(" << __LINE__ << ")" << std::endl; \
boost::filesystem::path file = FileManager::getSingletonPtr()->getGlobalDirectoryPath(); \
file /= "GlobalMPIError.log"; \
std::fstream f; \
f.open(file.string().c_str(), std::ios_base::app | std::ios_base::out); \
int rank; MPI_Comm_rank(MPI_COMM_WORLD,&rank); \
f << "ERROR in ProcessRank: " << rank << " @ " <<std::endl<< _message_ << std::endl << __FILE__ << "(" << __LINE__ << ")" << std::endl; \
f.close(); \
MPI_Abort(MPI_COMM_WORLD, -1); \
}

#define ERRORMSGMPI2( _message1_ , _message2_ ) ERRORMSGMPI( _message1_ << _message2_ )

#ifndef NDEBUG
	/**
	* @brief An Assert Macro to use within C++ code.
	* @param condition The condition which needs to be true otherwise an assertion is thrown!
	* @param message The message in form of cout out expression like: ÒVariable:Ó<<i<< Òhas failedÓ
	*/
    #define ASSERTMPIERROR( error_code , message ) { \
        if(error_code != MPI_SUCCESS){  \
            char * string; \
            int length; \
            MPI_Error_string( error_code , string, &length ); \
            ERRORMSGMPI2( string , message ) \
        } \
    }

#else
	#define ASSERTMPIERROR(condition,message) (void)0
#endif


#endif

