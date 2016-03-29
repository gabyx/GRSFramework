// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include "GRSF/common/CommonFunctionsOgre.hpp"


Ogre::StringVector OgreUtilities::convertToOgreStringVector(const std::vector<std::string> & strings) {
    Ogre::StringVector vec;

    for(int i=0; i<strings.size(); i++) {
        vec.push_back(strings[i]);
    }
    return vec;
};
