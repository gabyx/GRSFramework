#ifndef GRSF_Common_foreach_macro_hpp
#define GRSF_Common_foreach_macro_hpp

#if defined(__GNUC__) && __GNUC__==4 && __GNUC_MINOR__<4
#define FOREACH__HPP_TYPEOF 1
#elif defined(__GNUC__) && __GNUC__==4 && __GNUC_MINOR__>=4
#define FOREACH__HPP_AUTO 1
#elif defined(_MSC_VER) && _MSC_VER<1600
#include <boost/typeof/typeof.hpp>
#define FOREACH__HPP_TYPEOF 1
#define __typeof__(ex) BOOST_TYPEOF(ex)
#elif defined(_MSC_VER) && _MSC_VER>=1600
#define FOREACH__HPP_AUTO 1
#else
#error unknown compiler!
#endif

#if defined(FOREACH__HPP_TYPEOF)

#ifndef foreach
/** a for loop to iterate over a container.
    @param c (STL) container, has to support begin() and end() methods.
    @param i name of iterator which is available in loop body.
*/
#define foreach(c,i) for(/*lint --e{953}*/__typeof__((c).begin()) i = (c).begin(), end##i = (c).end(); i != end##i; ++i)
#endif

#ifndef foreach_r
/** a for loop to iterate over a container in reverse direction.
    @param c (STL) container, has to support rbegin() and rend() methods.
    @param i name of reverse iterator which is available in loop body.
*/
#define foreach_r(c,i) for(/*lint --e{953}*/__typeof__((c).rbegin()) i = (c).rbegin(), end##i = (c).rend(); i != end##i; ++i)
#endif

#ifndef foreach_e

/** a for loop to iterate over a container. You can call erase() on i
    if the erase() function does not invalidate iterators different
    from i. This is true for STL map, multimap, set, multiset.

    See http://llg.cubic.org/docs/stlerase.html for details.

    @param c (STL) container, has to support begin() and end() methods.
    @param i name of iterator which is available in loop body
*/
#define foreach_e(c,i) for(/*lint --e{953}*/__typeof__((c).end()) end##i = (c).end(), \
					      next##i = (c).begin(), \
					      i = (next##i==end##i)?next##i:next##i++; \
					    i != next##i; \
					    i = (next##i==end##i)?next##i:next##i++)
#endif

#elif defined(FOREACH__HPP_AUTO)

#define foreach(c,i) for(auto i = (c).begin(), end##i = (c).end(); i != end##i; ++i)
#define foreach_r(c,i) for(auto i = (c).rbegin(), end##i = (c).rend(); i != end##i; ++i)

#define foreach_e(c,i) for(auto end##i = (c).end(), next##i = (c).begin(), i = (next##i==end##i)?next##i:next##i++; \
			   i != next##i; \
			   i = (next##i==end##i)?next##i:next##i++)

#else
#error no code style defined!
#endif

#endif // HPP