#ifndef NeighbourData_hpp
#define NeighbourData_hpp

#include <vector>
#include <list>

#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include "RigidBody.hpp"




template<typename TDynamicsSystemConfig>
class NeighbourData {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef TDynamicsSystemConfig DynamicsSystemConfig;
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES_OF(TDynamicsSystemConfig)

    struct ExtRemoteData{
        RigidBodyType * m_body;

    };

    struct IntRemoteData{
        RigidBodyType * m_body;
        enum {NOT_NOTFIED, NOTIFIED, MOVE};
    };

    typedef ExtRemoteData ExtDataType;
    typedef IntRemoteData IntDataType;

    std::map<RigidBodyType::RigidBodyIdType, ExtDataType * > m_extRemoteBodies; // All external remote bodies
        // These bodies need an update from the neighbour.
        // If no update move body to m_garbageBodyList list

    std::map<RigidBodyType::RigidBodyIdType, IntDataType * > m_intRemoteBodies; // All external remote bodies
        // NOT_NOTIFIED: send whole body to the neighbour (change to NOTIFIED)
        // NOTIFIED: send update to neighbour
        // MOVE: send whole body to neighbour (put into m_garbageBodyList)

    void cleanUp();

private:

    std::list<RigidBodyType::RigidBodyType *> m_garbageBodyList;

    // We sent an Update for thesekind of bodies
    unsigned int m_ProcessId;
}
