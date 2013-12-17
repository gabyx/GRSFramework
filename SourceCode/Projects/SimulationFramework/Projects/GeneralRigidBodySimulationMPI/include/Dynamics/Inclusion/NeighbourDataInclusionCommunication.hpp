#ifndef NeighbourDataInclusionCommunication_hpp
#define NeighbourDataInclusionCommunication_hpp


#include <vector>
#include <list>
#include <type_traits>


#include "TypeDefs.hpp"

#include "ContactGraphNodeDataMPI.hpp"


namespace NeighbourDataInclusionCommunication_impl{

    struct RemoteData{
        DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
        RemoteData(RigidBodyType * body):m_pBody(body){};
        RigidBodyType * const m_pBody;
    };
    struct LocalData{
        DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
        LocalData(RigidBodyType * body):m_pBody(body),m_pSplitBodyNode(NULL){};
        RigidBodyType * const m_pBody;
        ContactGraphNodeDataSplitBody * m_pSplitBodyNode;
    };

};

/**
* @brief This class is used in the NeighbourMap class as data structure for the communication in the inclusion solver
*/class NeighbourDataInclusionCommunication:  public NeighbourData< NeighbourDataInclusionCommunication_impl::LocalData,
                                                                    NeighbourDataInclusionCommunication_impl::RemoteData>
{
public:
    DEFINE_MPI_INFORMATION_CONFIG_TYPES
private:
    typedef NeighbourData< NeighbourDataInclusionCommunication_impl::LocalData,
                           NeighbourDataInclusionCommunication_impl::RemoteData> NeighbourDataDerived;
public:

    NeighbourDataInclusionCommunication(const RankIdType &neighbourRank): NeighbourDataDerived(neighbourRank){};

    typedef NeighbourDataDerived::RemoteIterator RemoteIterator;
    typedef NeighbourDataDerived::LocalIterator LocalIterator;

    typedef NeighbourDataDerived::LocalDataType LocalDataType;
    typedef NeighbourDataDerived::RemoteDataType RemoteDataType;

    void clear(){
        m_localBodies.clear();
        m_remoteBodies.clear();
    }
};


#endif // NeighbourDataInclusionCommunication_hpp

