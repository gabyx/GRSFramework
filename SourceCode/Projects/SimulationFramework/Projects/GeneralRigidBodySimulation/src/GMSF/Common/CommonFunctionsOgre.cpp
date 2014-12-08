/*
 *  CommonFunctions.cpp
 *
 *  Created by Gabriel NÃ¼tzi on 21.03.10.
 *  Copyright 2010 ETH. All rights reserved.
 *
 */

#include <GMSF/Common/CommonFunctionsOgre.hpp>


Ogre::StringVector OgreUtilities::convertToOgreStringVector(const std::vector<std::string> & strings) {
    Ogre::StringVector vec;

    for(int i=0; i<strings.size(); i++) {
        vec.push_back(strings[i]);
    }
    return vec;
};
