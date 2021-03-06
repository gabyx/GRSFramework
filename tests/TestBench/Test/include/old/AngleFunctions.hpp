// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef AngleFunctions_hpp
#define AngleFunctions_hpp

#include <cmath>

#include "TypeDefs.hpp"

namespace AngleFunctions
{
DEFINE_MATRIX_TYPES

/** Maps angle to the range [-pi,pi]
    * Sawtooth function as g = f(x+a/2) - a/2 , f = x - floor(x/a)*a , a = 2*pi
    */
inline PREC mapToPi(PREC x)
{
    return x - std::floor(x / (2.0 * M_PI) + 0.5) * M_PI * 2.0;
}

/** Maps angle to the range [0,2*pi]
    * Sawtooth function as f = x - floor(x/a)*a , a = 2*pi
    */
inline PREC mapTo2Pi(PREC x)
{
    return x - std::floor(x / (2.0 * M_PI)) * M_PI * 2.0;
}

/** Relative angle measured from angle */
inline PREC relativeAnglePi(PREC angle, PREC angle2)
{
    return mapToPi(angle2 - angle);
}
/** Relative angle measured from angle */
inline PREC relativeAngle2Pi(PREC angle, PREC angle2)
{
    return mapTo2Pi(angle2 - angle);
}
};

#endif  // AngleFunctions_hpp
