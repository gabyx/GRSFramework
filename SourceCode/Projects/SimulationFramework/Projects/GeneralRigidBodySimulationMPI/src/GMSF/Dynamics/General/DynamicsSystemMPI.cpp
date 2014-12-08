
#include "DynamicsSystemMPI.hpp"

#include "AddGyroTermVisitor.hpp"
#include "VectorToSkewMatrix.hpp"
#include "CommonFunctions.hpp"
#include "RigidBodyFunctions.hpp"
#include "QuaternionHelpers.hpp"

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

