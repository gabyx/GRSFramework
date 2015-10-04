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
    void initSimInfo(std::size_t nBodies,std::size_t nStates);
    void initFrame(boost::filesystem::path folder, std::string filename, double time, unsigned int frameNr);
    void addBodyState(RigidBodyStateAdd * s){};

//    template<typename StateContainer>
//    void addState(StateContainer & states);

    void finalizeFrame();

    inline bool isStopBodyLoop(){return false;}
    inline bool isStopFrameLoop(){return false;}
    inline bool isStopFileLoop(){return false;}
    /** ========================================================================*/

private:
    GridExtractionSettings m_settings;

    std::size_t m_nBodies;
    std::size_t m_nStates;
    boost::filesystem::path m_folder;
    std::string m_filename;
    double m_time;
    unsigned int m_frameNr;
};



#endif

