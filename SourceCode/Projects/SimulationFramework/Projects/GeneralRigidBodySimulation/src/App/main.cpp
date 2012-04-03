#include <iostream>

using namespace std;

#include <App.hpp>

#include <TinyOgre.hpp>
#include <MinimalOgre.hpp>

//#if OGRE_PLATFORM == PLATFORM_WIN32 || OGRE_PLATFORM == OGRE_PLATFORM_WIN32
//#define WIN32_LEAN_AND_MEAN
//#include "windows.h"

//INT WINAPI WinMain(HINSTANCE hInst, HINSTANCE, LPSTR strCmdLine, INT)
//#else
int main(int argc, char **argv)
//#endif
{

// // Create application object
//        MinimalOgre app;
//
//        try {
//            app.go();
//        } catch( Ogre::Exception& e ) {
//#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
//            MessageBox( NULL, e.getFullDescription().c_str(), "An exception has occured!", MB_OK | MB_ICONERROR | MB_TASKMODAL);
//#else
//            std::cerr << "An exception has occured: " <<
//                e.getFullDescription().c_str() << std::endl;
//#endif
//        }
//
//        return 0;



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
