#include "GRSF/Dynamics/General/DynamicsSystemBase.hpp"

#include "GRSF/Dynamics/General/VectorToSkewMatrix.hpp"
#include "GRSF/Dynamics/General/AddGyroTermVisitor.hpp"
#include "GRSF/Common/CommonFunctions.hpp"
#include "GRSF/Dynamics/General/RigidBodyFunctions.hpp"


DynamicsSystemBase::DynamicsSystemBase(){
    // set reasonable standart values:
   resetEnergy();
}

DynamicsSystemBase::~DynamicsSystemBase() {
    DECONSTRUCTOR_MESSAGE

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

