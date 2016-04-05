// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include "GRSF/dynamics/general/DynamicsSystemMPI.hpp"

#include "GRSF/dynamics/general/AddGyroTermVisitor.hpp"
#include "GRSF/dynamics/general/VectorToSkewMatrix.hpp"
#include "GRSF/common/CommonFunctions.hpp"
#include "GRSF/dynamics/general/RigidBodyFunctions.hpp"
#include "GRSF/dynamics/general/QuaternionHelpers.hpp"

DynamicsSystemMPI::DynamicsSystemMPI(){
}


DynamicsSystemMPI::~DynamicsSystemMPI() {
    DESTRUCTOR_MESSAGE

    // Delete all RigidBodys
    m_remoteSimBodies.deleteAllBodies();
};


const DynamicsSystemMPI::TopologyBuilderSettingsType & DynamicsSystemMPI::getSettingsTopoBuilder() const{
    return m_settingsTopologyBuilder;
}


void DynamicsSystemMPI::deleteSimBodies()
{
     m_simBodies.deleteAllBodies();
     m_remoteSimBodies.deleteAllBodies();
}

