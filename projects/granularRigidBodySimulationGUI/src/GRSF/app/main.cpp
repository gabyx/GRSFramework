// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include <iostream>
#include <sstream>

#include "GRSF/common/ApplicationCLOptions.hpp"
#include "GRSF/common/ApplicationSignalHandler.hpp"

#include "GRSF/app/App.hpp"

void callBackExit(int signum)
{
    std::cerr << "GRSFramework Sim: received signal: " << signum << " -> exit ..." << std::endl;
    GRSF_THROW_SIGNALEXCEPTION("GRSFramework Sim: received signal: " << signum << " -> exit ...");
}

//#if OGRE_PLATFORM == PLATFORM_WIN32 || OGRE_PLATFORM == OGRE_PLATFORM_WIN32
//#define WIN32_LEAN_AND_MEAN
//#include "windows.h"

// INT WINAPI WinMain(HINSTANCE hInst, HINSTANCE, LPSTR strCmdLine, INT)
//#else
int main(int argc, char** argv)
//#endif
{
    INSTANCIATE_UNIQUE_SINGELTON_CTOR(ApplicationSignalHandler, sigHandler, ({SIGINT, SIGUSR2, SIGPIPE}))
    sigHandler->registerCallback({SIGINT, SIGUSR2, SIGPIPE}, callBackExit, "callBackExit");

    // Parsing Input Parameters===================================
    INSTANCIATE_UNIQUE_SINGELTON(ApplicationCLOptions, opts)
    ApplicationCLOptions::getSingleton().parseOptions(argc, argv);
    // End Parsing =================================

    try
    {
        App demo;
        demo.startApp();
    }
    catch (SignalException& ex)
    {
        std::cerr << "SignalException occured: " << ex.what() << std::endl;
        std::cerr << "Exiting ..." << std::endl;
        exit(EXIT_SUCCESS);
    }
    catch (Exception& ex)
    {
        std::cerr << "Exception occured: " << ex.what() << std::endl;
        std::cerr << "Exiting ..." << std::endl;
        exit(EXIT_FAILURE);
    }
    catch (std::exception& ex)
    {
        std::cerr << "std::exception occured: " << ex.what() << std::endl;
        std::cerr << "Exiting ..." << std::endl;
        exit(EXIT_FAILURE);
    }
    catch (...)
    {
        std::cerr << "Unknown exception occured!" << std::endl;
        std::cerr << "Exiting ..." << std::endl;
        exit(EXIT_FAILURE);
    }

    return EXIT_SUCCESS;
}
