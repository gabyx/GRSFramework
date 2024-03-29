// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_buffers_StateRecorderProcess_hpp
#define GRSF_dynamics_buffers_StateRecorderProcess_hpp

#include <sstream>
#include <string>

#include <cerrno>
#include <cstring>

#include <unordered_map>

#include "GRSF/common/LogDefines.hpp"
#include "GRSF/common/TypeDefs.hpp"

#include "GRSF/singeltons/FileManager.hpp"

#include "GRSF/common/CommonFunctions.hpp"

#include "GRSF/common/SimpleLogger.hpp"
#include "GRSF/dynamics/general/MultiBodySimFilePart.hpp"

#include "GRSF/common/GetFileDescriptorInfo.hpp"

/**
* @ingroup StatesAndBuffers
* @brief This is the StateRecorder class which records each body's states to one process own MultiBodySimFilePart.
* @{
*/

class StateRecorderProcess
{
public:
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    StateRecorderProcess(unsigned int id = 0, unsigned int bufferSize = 100 * 1024 * 1024);
    ~StateRecorderProcess();

    void write(PREC time, const typename DynamicsSystemType::RigidBodySimContainerType& bodyList);

    void setDirectoryPath(boost::filesystem::path dir_path);

    bool createSimFile(bool truncate = true);
    bool closeSimFile();

protected:
    boost::filesystem::path m_directoryPath;  ///< The path where the sim body part file is opened!

    void getSimFilePartName(std::stringstream& s);

    Logging::Log* m_pSimulationLog;

    unsigned int m_accessId;
    unsigned long long m_bufferSize;

    MultiBodySimFilePart m_binarySimFile;
};

/** @} */

StateRecorderProcess::StateRecorderProcess(unsigned int id, unsigned int bufferSize)
    : m_accessId(id), m_bufferSize(bufferSize), m_binarySimFile(m_bufferSize)
{
    // Check if LogManager is available
    Logging::LogManager& manager = Logging::LogManager::getSingleton();
    m_pSimulationLog             = manager.getLog("SimulationLog");
    if (!m_pSimulationLog)
    {
        // Log does not exist make a new standart log!
        boost::filesystem::path filePath = FileManager::getSingleton().getLocalDirectoryPath();
        filePath /= GLOBAL_LOG_FOLDER_DIRECTORY;
        filePath /= "StateRecorderLog.log";
        m_pSimulationLog = manager.createLog("StateRecorderLog", true, true, filePath);
    }
}

StateRecorderProcess::~StateRecorderProcess()
{
    DESTRUCTOR_MESSAGE
    m_binarySimFile.close();
}

void StateRecorderProcess::setDirectoryPath(boost::filesystem::path dir_path)
{
    m_directoryPath = dir_path;
}

bool StateRecorderProcess::createSimFile(bool truncate)
{
    boost::filesystem::path file;
    std::stringstream s;

    file = m_directoryPath;
    getSimFilePartName(s);
    file /= s.str();

    if (!m_binarySimFile.openWrite(
            file, LayoutConfigType::LayoutType::NDOFqBody, LayoutConfigType::LayoutType::NDOFuBody, truncate))
    {
        LOG(m_pSimulationLog, "---> StateRecorderProcess:: Could not open SimFile: " << file.string() << std::endl;);
        LOG(m_pSimulationLog, m_binarySimFile.getErrorString());
        return false;
    }
    if (truncate)
    {
        LOG(m_pSimulationLog, "---> StateRecorderProcess:: Added SimFile (truncated):" << file.string() << std::endl;);
    }
    else
    {
        LOG(m_pSimulationLog, "---> StateRecorderProcess:: Added SimFile: " << file.string() << std::endl;);
    }
    return true;
}

void StateRecorderProcess::getSimFilePartName(std::stringstream& s)
{
    s.str("");
    s << "SimDataProcess"
      << "-" << m_accessId << SIM_FILE_PART_EXTENSION;
}

void StateRecorderProcess::write(PREC time, const typename DynamicsSystemType::RigidBodySimContainerType& bodyList)
{
    m_binarySimFile.write(time, bodyList);
}

bool StateRecorderProcess::closeSimFile()
{
    m_binarySimFile.close();
    return true;
}

#endif
