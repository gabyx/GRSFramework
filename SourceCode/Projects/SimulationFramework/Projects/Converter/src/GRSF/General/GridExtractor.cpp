
#include "GRSF/General/GridExtractor.hpp"
#include "GRSF/Common/HDF5Helpers.hpp"

GridExtractor::GridExtractor(GridExtractionSettings * settings,
                             Logging::Log * log)
: m_settings(settings), m_log(log)
{

    // Make Grid
    m_grid.reset( new GridType(m_settings->m_aabb,m_settings->m_dimension, m_settings->m_R_KI.transpose() ) );

    // setup overall data buffer (for all extractors)
    auto totalBytes = m_settings->resizeBuffer();
    LOGGCLEVEL1(m_log,"---> Make buffer for "<< m_settings->extractorCount() << " extractors:" << ((double)totalBytes / (1<< 20))<< " [mb]" << std::endl;);

}



GridExtractor::~GridExtractor()
{
    closeFile();
    // release of m_h5File closes file
}

void GridExtractor::initState(boost::filesystem::path filePath,
                              double time, unsigned int frameNr)
{
//    m_filePath = filePath;
    m_time = time;
    m_frameNr = frameNr;
}

void GridExtractor::initSimInfo(boost::filesystem::path simFile,
                                boost::filesystem::path filePath,
                                std::size_t nBodies,std::size_t nStates)
{
    m_nBodies = nBodies;
    m_nStates = nStates;

    LOGGCLEVEL1(m_log,"---> Init for new sim file:" << simFile << std::endl << "\toutputFile: " << filePath << std::endl;);

    // Make new path for hdf5 file (use the filePath from the converter and append gridFileName)
    boost::filesystem::path f = filePath; // /folder/File
    if(f.has_filename()){
        f += m_settings->m_fileName;          // /folder/FileGridA.h5 for example
    }else{
        f /= m_settings->m_fileName;
    }

    // Open a new h5 file only if this is really a new file
    if( f != m_currentFilePath ){

        closeFile();

        LOGGCLEVEL1(m_log,"---> Opening hdf5 file: " << f << " (directory creation)" << std::endl);

        // Create all directories
        boost::filesystem::create_directories(f.parent_path());

        try{
            m_h5File.reset( new H5::H5File(f.string(), H5F_ACC_TRUNC) );
        }
        catch( H5::FileIException & e){
            m_h5File.reset(nullptr);
            ERRORMSG("File could not be opened: " << e.getDetailMsg() )
        }
        // Set current path
        m_currentFilePath = f;

        // Write grid data
        writeGridSettings();

        // Create States group
        m_statesGroup = m_h5File->createGroup("/States");
        m_filesGroup  = m_h5File->createGroup("/Files");

    }else{
        LOGGCLEVEL1(m_log,"---> Use already opened hdf5 file: " << f << std::endl);
    }

    // Create sim file group (/Files/SimFile0 , /Files/SimFile1 .... )
    m_currentSimFileGroup = m_filesGroup.createGroup("SimFile" + std::to_string(m_simFileCounter++));
    Hdf5Helpers::saveAttribute(m_currentSimFileGroup,simFile.filename().string(),"filePath");
    m_currentStateRefs = &m_stateRefs[m_currentSimFileGroup]; // stays valid also when rehash!
}

void GridExtractor::finalizeState(){

}


void GridExtractor::writeGridSettings(){

    LOGGCLEVEL1(m_log,"---> Write settings ..." << std::endl;);
    // Write the grid settings to the file
    H5::Group g = m_h5File->createGroup("/GridSettings");

    auto o = Hdf5Helpers::saveData(g, m_settings->m_aabb,"OOBB");

    Hdf5Helpers::saveData(o, m_settings->m_R_KI,"R_KI" );
    Hdf5Helpers::saveData(g, m_settings->m_dimension,"dimensions" );

    auto b = g.createGroup("./Bounds");
    Hdf5Helpers::saveData(b, m_settings->m_minPointOrig, "minPoint" );
    Hdf5Helpers::saveData(b, m_settings->m_maxPointOrig, "maxPoint" );
}


void GridExtractor::closeFile(){
    if(!m_currentFilePath.empty()){
            // finish opened file!!!
            // write all references for each simFiles
            writeReferences();
            m_h5File.reset(nullptr);
            m_currentFilePath = "";

            // reset all counters
            m_stateCounter = 0;
            m_simFileCounter = 0;
            m_stateRefs.clear();
            m_currentStateRefs = nullptr;
    }
}

void GridExtractor::writeReferences(){
    // Write all references
    // write for each state a reference to the corresponding simfile group

    if(!m_stateRefs.empty()){
        LOGGCLEVEL1(m_log,"---> Write state reference for each sim file ..." << std::endl;);
    }

    for(auto & p : m_stateRefs){
        Hdf5Helpers::saveData(p.first, p.second, "StateRefs");
    }

}


