// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_common_TypenameComparision_hpp
#define GRSF_common_TypenameComparision_hpp

struct FalseType
{
    static const bool value = false;
};
struct TrueType
{
    static const bool value = true;
};

template <typename T1, typename T2>
struct IsSame
{
    static const bool result = false;
};

template <typename T>
struct IsSame<T, T>
{
    static const bool result = true;
};

#endif