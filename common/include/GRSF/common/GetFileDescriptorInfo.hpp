// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_common_GetFileDescriptorInfo_hpp
#define GRSF_common_GetFileDescriptorInfo_hpp

// If you get a compile time error here,
//you may have not implemented to get some file descriptor info for your system!

#ifdef _WIN64
   //define something for Windows (64-bit)
#elif _WIN32
   //define something for Windows (32-bit)
#elif __APPLE__
    #include "TargetConditionals.h"
    #if TARGET_OS_IPHONE && TARGET_IPHONE_SIMULATOR
        // define something for simulator
    #elif TARGET_OS_IPHONE
        // define something for iphone
    #else
        #define TARGET_OS_OSX 1
        // define something for OSX
    #endif
#elif __linux

    #include <sys/resource.h>

    void getLimitOpenFiles(std::stringstream & s){
        rlimit rlim;
        getrlimit(RLIMIT_NOFILE, &rlim);
        s << "File Descriptor Limit: " << rlim.rlim_cur;
    }


#elif __unix // all unices not caught above
    // Unix
#elif __posix
    // POSIX
#endif


#endif
