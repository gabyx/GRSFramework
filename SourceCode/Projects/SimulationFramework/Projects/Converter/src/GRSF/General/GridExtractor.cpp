
#include "GRSF/General/GridExtractor.hpp"



void GridExtractor::initState(boost::filesystem::path folder,
                                      std::string filename,
                                      double time,
                                      unsigned int frameNr)
{
    m_folder = folder;
    m_filename = filename;
    m_time = time;
    m_frameNr = frameNr;
}

void GridExtractor::initSimInfo(std::size_t nBodies,std::size_t nStates){
    m_nBodies = nBodies;
    m_nStates = nStates;
}

void GridExtractor::finalizeState(){

}



