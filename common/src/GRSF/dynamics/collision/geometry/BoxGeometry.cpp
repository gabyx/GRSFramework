// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include "GRSF/dynamics/collision/geometry/BoxGeometry.hpp"
// Easy static char to build easily the points!
char BoxGeometry::m_pointIdx[8*3] ={ 1,1,1,
                                            -1,1,1,
                                            1,-1,1,
                                            -1,-1,1,
                                            1,1,-1,
                                            -1,1,-1,
                                            1,-1,-1,
                                            -1,-1,-1};
