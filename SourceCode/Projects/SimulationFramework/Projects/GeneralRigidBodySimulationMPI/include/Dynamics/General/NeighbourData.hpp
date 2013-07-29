#ifndef NeighbourData_hpp
#define NeighbourData_hpp

#include <vector>
#include <list>

#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include "RigidBody.hpp"
#include "RigidBodyContainer.hpp"



template<typename TDynamicsSystemConfig>
class NeighbourData {
public:

    typedef TDynamicsSystemConfig DynamicsSystemConfig;
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES_OF(TDynamicsSystemConfig)

    struct RemoteOverlapData{
        RigidBodyType * m_body;
    };

    struct LocalOverlapData{
        RigidBodyType * m_body;
        enum {NOT_NOTFIED, NOTIFIED, MOVE};
    };

    std::map<typename RigidBodyType::RigidBodyIdType, RemoteOverlapData > m_remoteBodies; // All overlapping remote bodies
        // These bodies need an update from the neighbour.
        // If no update move body to m_garbageBodyList list

    std::map<typename RigidBodyType::RigidBodyIdType, LocalOverlapData > m_localBodies; // All overlapping local bodies
        // NOT_NOTIFIED: send whole body to the neighbour (change to NOTIFIED)
        // NOTIFIED: send update to neighbour
        // MOVE: send whole body to neighbour (put into m_garbageBodyList)

    void cleanUp();

private:

};


#endif
