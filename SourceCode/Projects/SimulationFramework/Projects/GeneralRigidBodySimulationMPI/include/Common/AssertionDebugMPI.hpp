
#ifndef AssertionDebugMPI_H
#define AssertionDebugMPI_H

// Add an Assertion Debuggin!

//#define NDEBUG
#include <assert.h>
#include <stdlib.h>
#include <iostream>
#include <typeinfo>

#include <mpi.h>

#define ERRORMSGMPI(message) { \
std::cerr << "ERROR :  @ " <<std::endl<< message << std::endl << __FILE__ << " (" << __LINE__ << ")" << std::endl; abort();  \
MPI_Abort(MPI_COMM_WORLD, -1); \
}


#endif

