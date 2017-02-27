// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_common_SerializationHelpersTuple_hpp
#define GRSF_common_SerializationHelpersTuple_hpp

#include <tuple>
#include "GRSF/common/SfinaeMacros.hpp"

namespace grsf
{
namespace details
{
template <typename Archive, typename... Elements, std::size_t N = sizeof...(Elements), SFINAE_ENABLE_IF(N > 1)>
inline void serialize(Archive& ar, std::tuple<Elements...>& t)
{
    ar& std::get<N - 1>(t);
    serialize<N - 1>(ar, t);
}

template <typename Archive, typename... Elements, std::size_t N = sizeof...(Elements), SFINAE_ENABLE_IF(N == 1)>
inline void serialize(Archive& ar, std::tuple<Elements...>& t)
{
    ar& std::get<N - 1>(t);
}
};
};

namespace boost
{
namespace serialization
{
template <typename Archive, typename... Elements>
Archive& serialize(Archive& ar, std::tuple<Elements...>& t, const unsigned int version)
{
    grsf::details::serialize(ar, t);
    return ar;
}
};
};

#endif
