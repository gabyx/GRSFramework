/*
 *  CommonFunctions.cpp
 *
 *  Created by Gabriel NÃ¼tzi on 21.03.10.
 *  Copyright 2010 ETH. All rights reserved.
 *
 */

#include <CommonFunctions.hpp>

 bool Utilities::details::stringToTypeImpl::convert(bool & t, const std::string& s) {
    int a;
    if( convert<int>(a, s)) {
        if(a) {
            t = true;
            return true;
        } else {
            t = false;
            return true;
        }
    }

    if( s == "true" || s =="True" || s=="TRUE") {
        t = true;
        return true;
    } else if( s == "false" || s =="False" || s=="FALSE") {
        t = false;
        return true;
    }
    return false;
}
