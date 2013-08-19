#ifndef MPITopology_hpp
#define MPITopology_hpp

#include <mpi.h>

#include "AssertionDebug.hpp"
#include "TypeDefs.hpp"

#include "Collider.hpp"

namespace MPILayer {

template<typename TDynamicsSystem, typename TRankId>
class ProcessTopology {
public:

    typedef typename TDynamicsSystem::DynamicsSystemConfig DynamicsSystemConfig;
    typedef TDynamicsSystem DynamicsSystemType;
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES_OF(DynamicsSystemConfig)

    typedef TRankId RankIdType;
    typedef std::vector<RankIdType> NeighbourRankList;

    virtual bool belongsPointToProcess(const Vector3 & point, TRankId &neighbourProcessRank) const {
        ERRORMSG("The ProcessTopology::belongsPointToProcess has not been implemented!");
    }
    virtual bool belongsPointToProcess(const Vector3 & point) const {
        ERRORMSG("The ProcessTopology::belongsPointToProcess has not been implemented!");
    }

    virtual const std::vector<TRankId> & getNeigbourRanks() const {
        ERRORMSG("The ProcessTopology::belongsPointToProcess has not been implemented!");
    }

    virtual bool belongsBodyToProcess(const RigidBodyType * body) const{
        ERRORMSG("The ProcessTopology::belongsBodyToProcess has not been implemented!");
    }

    virtual bool belongsBodyToProcess(const RigidBodyType * body, TRankId &neighbourProcessRank) const {
        ERRORMSG("The ProcessTopology::belongsBodyToProcess2 has not been implemented!");
    }

    virtual bool checkOverlap( RigidBodyType * body,
                                std::vector<TRankId> & neighbourProcessRanks,
                                bool & overlapsOwnProcess) {
        ERRORMSG("The ProcessTopology::checkOverlap has not been implemented!");
    }



    protected:
    unsigned int m_rank; ///< Own rank;
    NeighbourRankList m_nbRanks; ///< Neighbour ranks

};

// Prototype
template<typename TDynamicsSystem> class ProcessInformation;

template<typename TDynamicsSystem, typename TRankId>
class ProcessTopologyGrid : public ProcessTopology<TDynamicsSystem, TRankId> {
public:
    typedef typename TDynamicsSystem::DynamicsSystemConfig DynamicsSystemConfig;
    typedef TDynamicsSystem DynamicsSystemType;
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES_OF(DynamicsSystemConfig)

    typedef typename ProcessTopology<TDynamicsSystem, TRankId>::NeighbourRankList  NeighbourRankList;
    typedef TRankId RankIdType;

    typedef std::map<unsigned int, AABB<LayoutConfigType> > RankToAABBType;





    ProcessTopologyGrid(  const Vector3 & minPoint,
                          const Vector3 & maxPoint,
                          const MyMatrix<unsigned int>::Vector3 & dim,
                          unsigned int processRank);

    bool checkOverlap( RigidBodyType * body, std::vector<RankIdType> & neighbourProcessRanks, bool & overlapsOwnProcess);

    bool belongsBodyToProcess(const RigidBodyType * body) const {
        RankIdType nb;
        return belongsPointToProcess(body->m_r_S,nb);
    }

    bool belongsBodyToProcess(const RigidBodyType * body, RankIdType &neighbourProcessRank) const {
        return belongsPointToProcess(body->m_r_S,neighbourProcessRank);
    }

    bool belongsPointToProcess(const Vector3 & point) const {
        unsigned int nb;
        return belongsPointToProcess(point,nb);
    }


    bool belongsPointToProcess(const Vector3 & point, RankIdType &neighbourProcessRank) const;

    const NeighbourRankList & getNeigbourRanks() const {
        return this->m_nbRanks;
    }


private:

    RankToAABBType m_nbAABB;            ///< Neighbour AABB
    AABB<LayoutConfigType> m_aabb;      ///< Own AABB of this process
    CartesianGrid<LayoutConfigType,NoCellData> m_grid;

    Collider<DynamicsSystemType> m_Collider;
};


template<typename TDynamicsSystem, typename TRankId>
ProcessTopologyGrid<TDynamicsSystem,TRankId>::ProcessTopologyGrid(  const Vector3 & minPoint,
                          const Vector3 & maxPoint,
                          const MyMatrix<unsigned int>::Vector3 & dim,
                          unsigned int processRank): m_grid(minPoint,maxPoint, dim, ProcessInformation<TDynamicsSystem>::MASTER_RANK ) {
        this->m_rank = processRank;

        //Initialize neighbours
        this->m_nbRanks = m_grid.getCellNeigbours(this->m_rank);

        //Get all AABB's of all neighbours
        for(int i = 0; i < this->m_nbRanks.size(); i++) {
            m_nbAABB[ this->m_nbRanks[i] ] =  m_grid.getCellAABB(this->m_nbRanks[i]) ;
        }

        //Get AABB of own rank!
        m_aabb = m_grid.getCellAABB(this->m_rank);
};


template<typename TDynamicsSystem, typename TRankId>
bool ProcessTopologyGrid<TDynamicsSystem,TRankId>::belongsPointToProcess(const Vector3 & point, RankIdType &neighbourProcessRank) const {

    neighbourProcessRank = m_grid.getCellNumber(point);
    if(neighbourProcessRank == this->m_rank) {
        return true;
    }
    return false;
};


template<typename TDynamicsSystem, typename TRankId>
bool ProcessTopologyGrid<TDynamicsSystem,TRankId>::checkOverlap(RigidBodyType * body,
                                                                std::vector<RankIdType> & neighbourProcessRanks,
                                                                bool & overlapsOwnProcess) {
    // Check neighbour AABB
    typename RankToAABBType::const_iterator it;
    neighbourProcessRanks.clear();
    for(it = m_nbAABB.begin(); it != m_nbAABB.end(); it++) {
        if( m_Collider.checkOverlap(body,it->second) ) {
            neighbourProcessRanks.push_back(it->first);
        }
    }

    // Check own AABB
    overlapsOwnProcess = m_Collider.checkOverlap(body, m_aabb);

    return neighbourProcessRanks.size() > 0;
};

}; //MPILayer

#endif
