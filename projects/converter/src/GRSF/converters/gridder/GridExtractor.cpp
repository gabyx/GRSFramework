// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include "GRSF/converters/gridder/GridExtractor.hpp"
#include "GRSF/common/HDF5Helpers.hpp"

GridExtractor::GridExtractor(GridExtractionSettings* settings, Logging::Log* log) : m_settings(settings), m_log(log)
{
    // Make Grid
    m_grid.reset(new GridType(m_settings->m_aabb, m_settings->m_dimension, m_settings->m_R_KI.transpose()));

    // setup overall data buffer (for all extractors)
    auto totalBytes = m_settings->resizeBuffer();
    LOGGCLEVEL1(
        m_log,
        "---> Make buffer for " << m_settings->extractorCount() << " extractors:" << ((double)totalBytes / (1 << 20))
                                << " [mb]"
                                << std::endl;);
}

GridExtractor::~GridExtractor()
{
    closeFile();
    // release of m_h5File closes file
}

void GridExtractor::initState(boost::filesystem::path filePath, double time, std::size_t stateIdx)
{
    //    m_filePath = filePath;
    m_time     = time;
    m_stateIdx = stateIdx;
}

void GridExtractor::initSimInfo(boost::filesystem::path simFile,
                                boost::filesystem::path filePath,
                                std::size_t             nBodies,
                                std::size_t             nStates)
{
    m_globalStateOffset += m_nStates;  // add old states

    m_nBodies = nBodies;
    m_nStates = nStates;

    LOGGCLEVEL1(m_log,
                "---> Init for new sim file:" << simFile << std::endl
                                              << "\toutputFile: "
                                              << filePath
                                              << std::endl;);

    // Make new path for hdf5 file (use the filePath from the converter and append gridFileName)
    boost::filesystem::path f = filePath;  // /folder/File
    if (f.has_filename())
    {
        f += m_settings->m_fileName;  // /folder/FileGridA.h5 for example
    }
    else
    {
        f /= m_settings->m_fileName;
    }

    // Open a new h5 file only if this is really a new file
    if (f != m_currentFilePath)
    {
        closeFile();

        LOGGCLEVEL1(m_log, "---> Opening hdf5 file: " << f << " (directory creation)" << std::endl);

        // Create all directories
        boost::filesystem::create_directories(f.parent_path());

        try
        {
            m_h5File.reset(new H5::H5File(f.string(), H5F_ACC_TRUNC));
        }
        catch (H5::FileIException& e)
        {
            m_h5File.reset(nullptr);
            GRSF_ERRORMSG("File could not be opened: " << e.getDetailMsg())
        }
        // Set current path
        m_currentFilePath = f;

        // Write grid data
        writeGridSettings();

        // Create States group
        m_statesGroup = m_h5File->createGroup("/States");
        m_filesGroup  = m_h5File->createGroup("/Files");
    }
    else
    {
        LOGGCLEVEL1(m_log, "---> Use already opened hdf5 file: " << f << std::endl);
    }

    // Create sim file group (/Files/SimFile0 , /Files/SimFile1 .... )
    m_currentSimFileGroup = m_filesGroup.createGroup("SimFile" + std::to_string(m_simFileCounter++));
    Hdf5Helpers::saveAttribute(m_currentSimFileGroup, simFile.filename().string(), "filePath");

    Hdf5Helpers::saveAttribute(m_currentSimFileGroup, nStates, "nStates");
    Hdf5Helpers::saveAttribute(m_currentSimFileGroup, nBodies, "nBodies");
    Hdf5Helpers::saveAttribute(m_currentSimFileGroup, m_globalStateOffset, "m_globalStateOffset");

    m_currentStateRefs = &m_stateRefs[m_currentSimFileGroup];  // stays valid also when rehash!
}

void GridExtractor::finalizeState()
{
}

void GridExtractor::writeGridSettings()
{
    LOGGCLEVEL1(m_log, "---> Write settings ..." << std::endl;);
    // Write the grid settings to the file
    H5::Group g = m_h5File->createGroup("/GridSettings");

    auto o = Hdf5Helpers::saveData(g, m_settings->m_aabb, "OOBB");

    Hdf5Helpers::saveData(o, m_settings->m_R_KI, "R_KI");
    Hdf5Helpers::saveData(g, m_settings->m_dimension, "dimensions");

    auto b = g.createGroup("./Bounds");
    Hdf5Helpers::saveData(b, m_settings->m_minPointOrig, "minPoint");
    Hdf5Helpers::saveData(b, m_settings->m_maxPointOrig, "maxPoint");
}

void GridExtractor::closeFile()
{
    if (!m_currentFilePath.empty())
    {
        // finish opened file!!!
        // write all references for each simFiles
        writeReferences();
        m_h5File.reset(nullptr);
        m_currentFilePath = "";

        // reset all counters
        m_stateCounter   = 0;
        m_simFileCounter = 0;
        m_stateRefs.clear();
        m_currentStateRefs = nullptr;
    }
}

void GridExtractor::writeReferences()
{
    // Write all references
    // write for each state a reference to the corresponding sim file group

    if (!m_stateRefs.empty())
    {
        LOGGCLEVEL1(m_log, "---> Write state reference for each sim file ..." << std::endl;);
    }
    for (auto& p : m_stateRefs)
    {
        Hdf5Helpers::saveRefData(p.first, p.second, "StateRefs");
    }

    // Write a reference to the state group linking to the sim file
    // TODO does not work so far,
    //    for(auto & p : m_stateRefs){
    //        hobj_ref_t linkToFile;
    //        //p.first.reference(&linkToFile); // /Files/SimFile_i
    //        m_h5File->reference(&linkToFile,p.first,H5R_OBJECT);
    //        for(auto & stateLink: p.second){ // iterate over all states and link sim file in state group
    //            H5::Group stateG(*m_h5File,&stateLink,H5R_OBJECT);       // get group to the state
    //            Hdf5Helpers::saveAttribute(stateG,linkToFile,"simFileRef");
    //        }
    //    }
}
