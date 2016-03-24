#ifndef GRSF_common_StaticAssert_hpp
#define GRSF_common_StaticAssert_hpp


#include <boost/static_assert.hpp>

// based on Boost's BOOST_STATIC_ASSERT
// see www.boost.org for details.

//
// Helper macro JOIN (based on BOOST_JOIN):
// The following piece of macro magic joins the two
// arguments together, even when one of the arguments is
// itself a macro (see 16.3.1 in C++ standard).  The key
// is that macro expansion of macro arguments does not
// occur in DO_JOIN2 but does in DO_JOIN.
//
#define JOIN( X, Y ) DO_JOIN( X, Y )
#define DO_JOIN( X, Y ) DO_JOIN2(X,Y)
#define DO_JOIN2( X, Y ) X##Y


// XXX nvcc 2.3 can't handle STATIC_ASSERT
#if defined(__CUDACC__) /* && (CUDA_VERSION < 30)*/

    #define STATIC_ASSERT( B ) BOOST_STATIC_ASSERT(B);
    #define STATIC_ASSERTM(B,COMMENT) STATIC_ASSERT(B);

#else

    #define STATIC_ASSERT( B ) BOOST_STATIC_ASSERT( (B) );
    #define STATIC_ASSERTM( B ,COMMENT) BOOST_STATIC_ASSERT_MSG( (B), COMMENT );

#endif





#endif
