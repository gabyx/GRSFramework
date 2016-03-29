// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef PLATFORM_DEFINES
#define PLATFORM_DEFINES



#if defined _WIN32 || defined _WIN64
    #define HOLD_SYSTEM { system("pause"); }
#elif defined __unix__  || defined __unix || defined __apple__
    #define HOLD_SYSTEM { \
    printf("Press 'Enter' to exit the program ..."); \
    while (getchar() != '\n'); \
    printf("\n\n");}
#endif


#if defined _WIN32 || defined _WIN64
    #include <windows.h>
    #define SLEEP(_ms_)  Sleep(_ms_);
#elif defined __unix__  || defined __unix || defined __apple__
    #include <unistd.h>
    #define SLEEP(_ms_)  sleep(_ms_);
#endif


#endif
