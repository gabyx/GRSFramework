
#include "GRSF/dynamics/general/DynamicsSystemMPI.hpp"

#include "GRSF/dynamics/general/AddGyroTermVisitor.hpp"
#include "GRSF/dynamics/general/VectorToSkewMatrix.hpp"
#include "GRSF/common/CommonFunctions.hpp"
#include "GRSF/dynamics/general/RigidBodyFunctions.hpp"
#include "GRSF/dynamics/general/QuaternionHelpers.hpp"

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

