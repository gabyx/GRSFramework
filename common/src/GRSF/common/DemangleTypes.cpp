// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include "GRSF/common/DemangleTypes.hpp"

#ifdef __GNUG__
    #include <cstdlib>
    #include <memory>
    #include <cxxabi.h>
#endif


namespace demangle{
    namespace details{

#ifdef __GNUG__
    std::string demangle(const char* name) {

        int status = -4; // some arbitrary value to eliminate the compiler warning

        // enable c++11 by passing the flag -std=c++11 to g++
        std::unique_ptr<char, void(*)(void*)> res {
            abi::__cxa_demangle(name, NULL, NULL, &status),
            std::free
        };

        return (status==0) ? res.get() : name ;
    }
#else
    // does nothing if not g++
    std::string demangle(const char* name) {
        return name;
    }
#endif

    };
};
