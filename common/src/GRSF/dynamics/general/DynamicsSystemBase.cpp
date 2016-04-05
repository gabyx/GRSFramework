// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include "GRSF/dynamics/general/DynamicsSystemBase.hpp"

#include "GRSF/dynamics/general/VectorToSkewMatrix.hpp"
#include "GRSF/dynamics/general/AddGyroTermVisitor.hpp"
#include "GRSF/common/CommonFunctions.hpp"
#include "GRSF/dynamics/general/RigidBodyFunctions.hpp"


DynamicsSystemBase::DynamicsSystemBase(){
    // set reasonable standart values:
   resetEnergy();
}

DynamicsSystemBase::~DynamicsSystemBase() {
    DESTRUCTOR_MESSAGE

    // Delete all RigidBodys
    m_simBodies.deleteAllBodies();
    m_staticBodies.deleteAllBodies();

};

void DynamicsSystemBase::initializeLog(Logging::Log* pLog) {
    m_pSolverLog = pLog;
    ASSERTMSG(m_pSolverLog != nullptr, "Logging::Log: nullptr!");
}


void DynamicsSystemBase::reset() {
    //reset all external forces
    m_externalForces.reset();
}

void DynamicsSystemBase::resetEnergy() {
    m_currentTotEnergy = 0;
    m_currentPotEnergy= 0;
    m_currentKinEnergy= 0;
    m_currentTransKinEnergy= 0;
    m_currentRotKinEnergy= 0;
    m_currentSpinNorm= 0;
}

