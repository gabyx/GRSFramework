// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_common_ExpandParameterPack_hpp
#define GRSF_common_ExpandParameterPack_hpp

#include <type_traits>

namespace ExpandParameterPack
{
/** Empty function */
struct expandType
{
    template <typename... T>
    expandType(T&&...)
    {
    }
};
};

/** Use this macro to call a function on parameter pack
*   For example:
*     template<typename...T>
*     void work(T&&... t){
*        // call print on all t's
*        EXPAND_PARAMETERPACK(print(t))
*     }
*
*   The macro creates an empty type expandType and initializing it with a a list of zeros.
*   Initialization guarantees evaluation order!! (if you would call an empty function instead, this does not guarantee
evaluation order)
*   Potential side effect: Compiler does not optimize out the empty non used expandType, but that will definitely not
happen!

*   Source: http://stackoverflow.com/questions/17339789/how-to-call-a-function-on-all-variadic-template-args
*/

#define EXPAND_PARAMETERPACK(PATTERN) ExpandParameterPack::expandType{0, ((PATTERN), void(), 0)...};

#endif
