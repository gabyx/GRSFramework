// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

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
#define JOIN(X, Y) DO_JOIN(X, Y)
#define DO_JOIN(X, Y) DO_JOIN2(X, Y)
#define DO_JOIN2(X, Y) X##Y

// XXX nvcc 2.3 can't handle GRSF_STATIC_ASSERT
#if defined(__CUDACC__) /* && (CUDA_VERSION < 30)*/

#define GRSF_STATIC_ASSERT(B) BOOST_STATIC_ASSERT(B);
#define GRSF_STATIC_ASSERTM(B, COMMENT) GRSF_STATIC_ASSERT(B);

#else

#define GRSF_STATIC_ASSERT(B) BOOST_STATIC_ASSERT((B));
#define GRSF_STATIC_ASSERTM(B, COMMENT) BOOST_STATIC_ASSERT_MSG((B), COMMENT);

#endif

#endif
