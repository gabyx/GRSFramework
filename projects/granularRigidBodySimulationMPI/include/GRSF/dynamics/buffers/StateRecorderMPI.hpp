// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_buffers_StateRecorderMPI_hpp
#define GRSF_dynamics_buffers_StateRecorderMPI_hpp

#include <mpi.h>

#include <string>
#include <sstream>

#include <cstring>
#include <cerrno>

#include "GRSF/common/AssertionDebug.hpp"
#include "GRSF/common/LogDefines.hpp"
#include "GRSF/common/TypeDefs.hpp"

#include "GRSF/singeltons/FileManager.hpp"

#include "GRSF/common/CommonFunctions.hpp"

#include "GRSF/common/SimpleLogger.hpp"

#include "GRSF/dynamics/general/MPICommunication.hpp"

#include "GRSF/dynamics/general/MultiBodySimFileMPI.hpp"

/**
* @ingroup StatesAndBuffers
* @brief This is the StateRecorder class which records each body's states to one process own MultiBodySimFilePart.
* @{
*/

class StateRecorderMPI {
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    StateRecorderMPI(unsigned int nSimBodies, std::shared_ptr<typename MPILayer::ProcessCommunicator::ProcessInfoType> pProcInfo);
    ~StateRecorderMPI();

    //Each process writes its stuff at a specific offset
    template< typename TRigidBodyContainer >
    void write(PREC time, const TRigidBodyContainer & bodyList);

    void setDirectoryPath(boost::filesystem::path dir_path);

    //Collective operation
    bool createSimFile(bool truncate = true);
    //Collective operation
    bool closeAll();

protected:

    unsigned int m_nSimBodies;
    MultiBodySimFileMPI m_fh;

    std::shared_ptr<typename MPILayer::ProcessCommunicator::ProcessInfoType> m_pProcInfo;

    boost::filesystem::path m_directoryPath; ///< The path where the sim body part file is opened!

    void getSimBodyFileName(std::stringstream & s);

    Logging::Log * m_pSimulationLog;



    //Write buffer
    //std::vector<char> m_writebuffer;
    //    boost::iostreams::back_insert_device<std::vector<char> >m_ins; // is initialized first
    //    boost::iostreams::stream< boost::iostreams::back_insert_device<std::vector<char> > > m_stream;  //is initialized second
    //    boost::archive::binary_oarchive m_oa; // is initialized third

};

/** @} */




StateRecorderMPI::StateRecorderMPI(unsigned int nSimBodies,
                                   std::shared_ptr<typename MPILayer::ProcessCommunicator::ProcessInfoType> pProcInfo):
    m_nSimBodies(nSimBodies),
    m_pProcInfo(pProcInfo)
//    ,m_ins(m_writebuffer),
//    m_stream(m_ins),
//    m_oa( m_stream,boost::archive::no_codecvt | boost::archive::no_header)
{
    //Check if LogManager is available
    Logging::LogManager & manager = Logging::LogManager::getSingleton();
    m_pSimulationLog = manager.getLog("SimulationLog");
    if(!m_pSimulationLog) {
        // Log does not exist make a new standart log!
        boost::filesystem::path filePath = FileManager::getSingleton().getLocalDirectoryPath();
        filePath /= GLOBAL_LOG_FOLDER_DIRECTORY;
        filePath /= "StateRecorderLog.log";
        m_pSimulationLog = manager.createLog("StateRecorderLog",true,true,filePath);
    }

}


StateRecorderMPI::~StateRecorderMPI() {



}


void StateRecorderMPI::setDirectoryPath(boost::filesystem::path dir_path){
    m_directoryPath = dir_path;
}


bool StateRecorderMPI::createSimFile(bool truncate){

    boost::filesystem::path file;
    std::stringstream s;

    file = m_directoryPath;
    getSimBodyFileName(s);
    file /= s.str();

    LOG(m_pSimulationLog,"MPI> StateRecorderMPI: Try Opened File : " << file << std::endl)
    //Collective (all processes do this)
    bool res = m_fh.openWrite(m_pProcInfo->getCommunicator(),file,NDOFqBody, NDOFuBody,m_nSimBodies);
    if(!res){
        ERRORMSG(m_fh.getErrorString());
    }
    LOG(m_pSimulationLog,"MPI> StateRecorderMPI: Opened File : " << file << std::endl)
    return true;
}


void StateRecorderMPI::getSimBodyFileName(std::stringstream & s){
    s.str("");
    s <<SIM_FILE_PREFIX<<SIM_FILE_MPI_EXTENSION;
}


template< typename TRigidBodyContainer >
void StateRecorderMPI::write(PREC time, const TRigidBodyContainer & bodyList){
    m_fh.write(time,bodyList);
}



bool StateRecorderMPI::closeAll(){
   m_fh.close();
   return true;
}

#endif


