// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_common_Delegates_hpp
#define GRSF_common_Delegates_hpp

#include <fastfunc/fastfunc.hpp>  // Use FastFunc
#include <functional>

namespace Delegates
{
/**
    * This file defines the used delegates in this framework
    */

template <typename T>
using DelegateFunc = typedef ssvu::FastFunc<T>;
// template<typename T> using DelegateFunc = typedef std::function<T>;
}

#endif  // Delegates_hpp
