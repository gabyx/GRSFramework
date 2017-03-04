// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_common_UnorderedContainerHelpers_hpp
#define GRSF_common_UnorderedContainerHelpers_hpp

/** Intersection and union function for unordered containers which support a fast lookup function find()
 *  Return values are moved by move-semantics, for c++11/c++14 this is efficient, otherwise it results in a copy
 */

namespace unorderedHelpers
{
template <typename UnorderedIn1, typename UnorderedIn2, typename UnorderedOut = UnorderedIn1>
UnorderedOut makeIntersection(const UnorderedIn1& in1, const UnorderedIn2& in2)
{
    if (in2.size() < in1.size())
    {
        return makeIntersection<UnorderedIn2, UnorderedIn1, UnorderedOut>(in2, in1);
    }

    UnorderedOut out;
    auto e = in2.end();
    for (auto& v : in1)
    {
        if (in2.find(v) != e)
        {
            out.insert(v);
        }
    }
    return out;
}

template <typename UnorderedIn1, typename UnorderedIn2, typename UnorderedOut = UnorderedIn1>
UnorderedOut makeUnion(const UnorderedIn1& in1, const UnorderedIn1& in2)
{
    UnorderedOut out;
    out.insert(in1.begin(), in1.end());
    out.insert(in2.begin(), in2.end());
    return out;
}
}

#endif
