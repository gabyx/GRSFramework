﻿#include <iostream>

using namespace std;

#include "ApplicationCLOptions.hpp"

#include <App.hpp>

//#if OGRE_PLATFORM == PLATFORM_WIN32 || OGRE_PLATFORM == OGRE_PLATFORM_WIN32
//#define WIN32_LEAN_AND_MEAN
//#include "windows.h"

//INT WINAPI WinMain(HINSTANCE hInst, HINSTANCE, LPSTR strCmdLine, INT)
//#else
int main(int argc, char **argv)
//#endif
{

    // Parsing Input Parameters===================================
    new ApplicationCLOptions();
    ApplicationCLOptions::getSingletonPtr()->parseOptions(argc,argv);
   // End Parsing =================================


	try
    {
		App demo;
		demo.startApp();
    }
	catch(std::exception& e)
    {
#if OGRE_PLATFORM == PLATFORM_WIN32 || OGRE_PLATFORM == OGRE_PLATFORM_WIN32
        MessageBoxA(NULL, e.what(), "An exception has occurred!", MB_OK | MB_ICONERROR | MB_TASKMODAL);
#else
        fprintf(stderr, "An exception has occurred: %s\n", e.what());
#endif
	}

   system("pause");
  return 0;
}
