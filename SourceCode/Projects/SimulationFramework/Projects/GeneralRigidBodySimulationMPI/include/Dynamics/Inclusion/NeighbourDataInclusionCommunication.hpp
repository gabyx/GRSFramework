#ifndef NeighbourDataInclusionCommunication_hpp
#define NeighbourDataInclusionCommunication_hpp


#include <vector>
#include <list>
#include <type_traits>


#include "TypeDefs.hpp"


/**
* @brief This class is used in the NeighbourMap class as data structure for the communication in the inclusion solver
*/
class NeighbourDataInclusionCommunication {
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
    DEFINE_MPI_INFORMATION_CONFIG_TYPES

    NeighbourDataInclusionCommunication(RankIdType rank): m_neighbourRank(rank){};

    void addRemoteBody(RigidBodyType * body){
        m_remoteBodiesWC.push_back(body->m_id);
    }

    void clear(){
        m_remoteBodiesWC.clear();
    }

    std::vector<RankIdType> & getRemoteBodiesWC(){ return m_remoteBodiesWC;}

private:
    RankIdType m_neighbourRank; // This is the rank to which this data structure belongs!

    /** all remote bodies which have contacts and belong to this rank */
    std::vector<RankIdType> m_remoteBodiesWC; //WC=with contact

};

#endif // NeighbourDataInclusionCommunication_hpp

