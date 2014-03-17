#ifndef StaticAssert_hpp
#define StaticAssert_hpp


#include <boost/static_assert.hpp>

// based on Boost's BOOST_STATIC_ASSERT
// see www.boost.org for details.

//
// Helper macro THRUST_JOIN (based on BOOST_JOIN):
// The following piece of macro magic joins the two
// arguments together, even when one of the arguments is
// itself a macro (see 16.3.1 in C++ standard).  The key
// is that macro expansion of macro arguments does not
// occur in THRUST_DO_JOIN2 but does in THRUST_DO_JOIN.
//
#define JOIN( X, Y ) DO_JOIN( X, Y )
#define DO_JOIN( X, Y ) DO_JOIN2(X,Y)
#define DO_JOIN2( X, Y ) X##Y


// XXX nvcc 2.3 can't handle STATIC_ASSERT
#if defined(__CUDACC__) /* && (CUDA_VERSION < 30)*/

    #define STATIC_ASSERT( B ) BOOST_STATIC_ASSERT(B);
    #define STATIC_ASSERT2(B,COMMENT) STATIC_ASSERT(B);

#else

    #define STATIC_ASSERT( B ) BOOST_STATIC_ASSERT( (B) );
    #define STATIC_ASSERT2( B ,COMMENT) STATIC_ASSERT( (B) );
    //#define STATIC_ASSERT( B )
    //#define STATIC_ASSERT2(B,COMMENT)
#endif





#endif