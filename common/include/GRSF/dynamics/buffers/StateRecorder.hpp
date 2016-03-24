// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_buffers_StateRecorder_hpp
#define GRSF_dynamics_buffers_StateRecorder_hpp

#include <string>

#include "GRSF/singeltons/FileManager.hpp"

#include "GRSF/common/TypeDefs.hpp"

#include "GRSF/common/CommonFunctions.hpp"
#include "GRSF/dynamics/buffers/DynamicsState.hpp"
#include "GRSF/common/LogDefines.hpp"
#include "GRSF/dynamics/general/MultiBodySimFile.hpp"
#include "GRSF/common/SimpleLogger.hpp"

#include DynamicsSystem_INCLUDE_FILE

/**
* @ingroup StatesAndBuffers
* @brief This is the StateRecorder class which records states to a MultiBodySimFile.
* @{
*/
class StateRecorder {
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    StateRecorder(const unsigned int nSimBodies);

    ~StateRecorder();
    bool createSimFile(boost::filesystem::path file_path);
    bool createSimFileCopyFromReference(boost::filesystem::path new_file_path, boost::filesystem::path ref_file_path);
    void closeAll();

    //void addState(const DynamicsState * state);
    /*void writeAllStates();*/

    void write(const DynamicsState* value);

    template<typename TRigidBodyContainer>
    void write(PREC time, const TRigidBodyContainer & bodyList) {
        m_binarySimFile.write(time, bodyList);
    }


    StateRecorder & operator << (const DynamicsState* value);

protected:

    Logging::Log * m_pSimulationLog;

    //void scanAllSimDataFiles(boost::filesystem::path path_name, bool with_SubDirs = false);
    // All files created have the name "<m_fileNamePrefix>_<m_folderIdCounter>.sim"
    // If files already exist in the folder m_folderIdCounter is set to the actual number which does not exist already!
    //std::vector<boost::filesystem::path> m_SimFilePaths;
    //Ogre::StringVector m_SimFileNames;


    MultiBodySimFile    m_binarySimFile;

    //std::vector< DynamicsState >	m_states;

    unsigned int m_nSimBodies;
};

/** @} */



#endif
