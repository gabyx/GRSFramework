
#include "GRSF/Dynamics/General/DynamicsSystemMPI.hpp"

#include "GRSF/Dynamics/General/AddGyroTermVisitor.hpp"
#include "GRSF/Dynamics/General/VectorToSkewMatrix.hpp"
#include "GRSF/Common/CommonFunctions.hpp"
#include "GRSF/Dynamics/General/RigidBodyFunctions.hpp"
#include "GRSF/Dynamics/General/QuaternionHelpers.hpp"

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

