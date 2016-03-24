#include <iostream>

using namespace std;

#include "GRSF/common/ApplicationCLOptions.hpp"

#include "GRSF/app/App.hpp"

//#if OGRE_PLATFORM == PLATFORM_WIN32 || OGRE_PLATFORM == OGRE_PLATFORM_WIN32
//#define WIN32_LEAN_AND_MEAN
//#include "windows.h"

//INT WINAPI WinMain(HINSTANCE hInst, HINSTANCE, LPSTR strCmdLine, INT)
//#else
int main(int argc, char **argv)
//#endif
{

    // Parsing Input Parameters===================================
    INSTANCIATE_UNIQUE_SINGELTON(ApplicationCLOptions,opts)
    ApplicationCLOptions::getSingleton().parseOptions(argc,argv);
    // End Parsing =================================


	try
    {
		App demo;
		demo.startApp();
    }
	catch(std::exception& e)
    {
#if OGRE_PLATFORM == PLATFORM_WIN32 || OGRE_PLATFORM == OGRE_PLATFORM_WIN32
        MessageBoxA(nullptr, e.what(), "An exception has occurred!", MB_OK | MB_ICONERROR | MB_TASKMODAL);
#else
        fprintf(stderr, "An exception has occurred: %s\n", e.what());
#endif
	}

  return 0;
}
