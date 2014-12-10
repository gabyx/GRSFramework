
#include "GMSF/Dynamics/General/DynamicsSystemMPI.hpp"

#include "GMSF/Dynamics/General/AddGyroTermVisitor.hpp"
#include "GMSF/Dynamics/General/VectorToSkewMatrix.hpp"
#include "GMSF/Common/CommonFunctions.hpp"
#include "GMSF/Dynamics/General/RigidBodyFunctions.hpp"
#include "GMSF/Dynamics/General/QuaternionHelpers.hpp"

DynamicsSystemMPI::DynamicsSystemMPI(){
}


DynamicsSystemMPI::~DynamicsSystemMPI() {
    DECONSTRUCTOR_MESSAGE

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

