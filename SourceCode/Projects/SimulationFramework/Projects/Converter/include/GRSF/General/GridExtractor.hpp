#ifndef GRSF_General_GridExtractor_hpp
#define GRSF_General_GridExtractor_hpp

#include <boost/filesystem.hpp>

#include "GRSF/Common/TypeDefs.hpp"
#include "GRSF/Common/LogDefines.hpp"

#include "GRSF/Dynamics/Buffers/RigidBodyState.hpp"

#include "GRSF/General/GridExtractionSettings.hpp"

class GridExtractor{
public:

    GridExtractor(const GridExtractionSettings & settings): m_settings(settings){}

  /** provide function for SimFileConverter ==================================*/
    void setup();
    void initSimInfo(std::size_t nBodies,std::size_t nStates);
    void initFrame(boost::filesystem::path folder, std::string filename, double time, unsigned int frameNr);
    void addBodyState(RigidBodyStateAdd * s);
    void finalizeFrame();

    bool isStopBodyLoop();
    bool isStopFrameLoop();
    bool isStopFileLoop();
    /** ========================================================================*/

private:
    GridExtractionSettings m_settings;

};



#endif

