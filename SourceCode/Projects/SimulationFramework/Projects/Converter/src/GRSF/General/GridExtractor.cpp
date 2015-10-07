
#include "GRSF/General/GridExtractor.hpp"
#include "GRSF/Common/HDF5Helpers.hpp"

GridExtractor::GridExtractor(GridExtractionSettings * settings,
                             Logging::Log * log)
: m_settings(settings), m_log(log)
{

    // Make Grid
    m_grid.reset( new GridType(m_settings->m_aabb,m_settings->m_dimension) );

    // setup overall data buffer (for all extractors)
    auto totalBytes = m_settings->resizeBuffer();
    LOGGCLEVEL1(m_log,"---> Make buffer for "<< m_settings->extractorCount() << " extractors:" << ((double)totalBytes / (1<< 20))<< " [mb]" << std::endl;);

    // Open h5 File
    LOGGCLEVEL1(m_log,"---> Opening hdf5 file: " << m_settings->m_fileName << std::endl;);
    m_h5File.reset( new H5::H5File(m_settings->m_fileName, H5F_ACC_TRUNC) );

    // Write grid data
    writeGridSettings();

}



GridExtractor::~GridExtractor()
{
    // release of m_h5File closes file
}

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


void GridExtractor::writeGridSettings(){

    LOGGCLEVEL1(m_log,"---> Write settings ..." << std::endl;);
    // Write the grid settings to the file
    H5::Group group = m_h5File->createGroup("/GridSettings");

    auto g = Hdf5Helpers::saveData(group, m_settings->m_aabb,"OOBB");

    Hdf5Helpers::saveData(g, m_settings->m_R_KI,"R_KI" );
    Hdf5Helpers::saveData(g, m_settings->m_dimension,"dimensions" );
    Hdf5Helpers::saveData(g, m_grid->getDx(),"dx" );
}


