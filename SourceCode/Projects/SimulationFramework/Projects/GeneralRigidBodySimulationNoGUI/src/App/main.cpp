#include <iostream>

#include "TypeDefs.hpp"
#include "FileManager.hpp"
#include "SimpleLogger.hpp"

#include "SimulationManager.hpp"


int main(int argc, char **argv)
{

    new Logging::LogManager();
    new FileManager();

   SimulationManager<GeneralConfig> mgr;

   mgr.setup();
   mgr.startSimThread();
   mgr.waitForSimThread();
   system("pause");
  return 0;
}

