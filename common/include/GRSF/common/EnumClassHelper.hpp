// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_common_EnumClassHelper_hpp
#define GRSF_common_EnumClassHelper_hpp

#include <type_traits>

namespace EnumConversion{
    /** This function casts any enum class to the underlying type */
    template<typename E>
    constexpr auto toIntegral(const E e) -> typename std::underlying_type<E>::type
    {
       return static_cast<typename std::underlying_type<E>::type>(e);
    }
};

#endif // EnumClassHelper_hpp



