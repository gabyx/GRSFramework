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

    virtual bool belongsPointToProcess(const Vector3 & point, TRankId &neighbourProcessRank) const {
        ERRORMSG("The ProcessTopology::belongsPointToProcess has not been implemented!");
    }
    virtual bool belongsPointToProcess(const Vector3 & point) const {
        ERRORMSG("The ProcessTopology::belongsPointToProcess has not been implemented!");
    }

    virtual const std::vector<TRankId> & getNeigbourRanks() const {
        ERRORMSG("The ProcessTopology::belongsPointToProcess has not been implemented!");
    }

    virtual bool belongsBodyToProcess(const RigidBodyType * body) const {
        ERRORMSG("The ProcessTopology::belongsBodyToProcess has not been implemented!");
    }

    virtual bool belongsBodyToProcess(const RigidBodyType * body, TRankId &neighbourProcessRank) const {
        ERRORMSG("The ProcessTopology::belongsBodyToProcess has not been implemented!");
    }

    virtual bool checkOverlap(const  RigidBodyType * body, std::vector<TRankId> & neighbourProcessRanks) const {
        ERRORMSG("The ProcessTopology::belongsBodyToProcess has not been implemented!");
    }

};

// Prototype
template<typename TDynamicsSystem> class ProcessInformation;

template<typename TDynamicsSystem, typename TRankId>
class ProcessTopologyGrid : public ProcessTopology<TDynamicsSystem, TRankId> {
public:
    typedef typename TDynamicsSystem::DynamicsSystemConfig DynamicsSystemConfig;
    typedef TDynamicsSystem DynamicsSystemType;
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES_OF(DynamicsSystemConfig)

    typedef std::map<unsigned int,AABB<LayoutConfigType> > RankToAABBType;
    typedef TRankId RankIdType;

    ProcessTopologyGrid(  const Vector3 & minPoint,
                          const Vector3 & maxPoint,
                          const MyMatrix<unsigned int>::Vector3 & dim,
                          unsigned int processRank);

    bool checkOverlap(const  RigidBodyType * body, std::vector<RankIdType> & neighbourProcessRanks) const;

    bool belongsBodyToProcess(const RigidBodyType * body) const {
        unsigned int nb;
        return belongsPointToProcess(body->m_r_S,nb);
    }

    bool belongsBodyToProcess(const RigidBodyType * body, RankIdType &neighbourProcessRank) const {
        return belongsPointToProcess(body->m_r_S,neighbourProcessRank);
    }

    bool belongsPointToProcess(const Vector3 & point) const {
        unsigned int nb;
        return belongsPointToProcess(point,nb);
    }


    bool belongsPointToProcess(const Vector3 & point, unsigned int &neighbourProcessRank) const;

    const std::vector<unsigned int> & getNeigbourRanks() const {
        return m_nbRanks;
    }


private:
    unsigned int m_rank; ///< Own rank;
    std::vector<unsigned int> m_nbRanks; ///< Neighbour ranks
    RankToAABBType m_nbAABB;            ///< Neighbour AABB
    CartesianGrid<LayoutConfigType,NoCellData> m_grid;

    Collider<DynamicsSystemType> m_Collider;
};


template<typename TDynamicsSystem, typename TRankId>
ProcessTopologyGrid<TDynamicsSystem,TRankId>::ProcessTopologyGrid(  const Vector3 & minPoint,
                          const Vector3 & maxPoint,
                          const MyMatrix<unsigned int>::Vector3 & dim,
                          unsigned int processRank): m_grid(minPoint,maxPoint, dim, ProcessInformation<TDynamicsSystem>::MASTER_RANK ) {
        m_rank = processRank;

        //Initialize neighbours
        m_nbRanks = m_grid.getCellNeigbours(m_rank);

        //Get all AABB's of all neighbours
        for(int i = 0; i < m_nbRanks.size(); i++) {
            m_nbAABB[ m_nbRanks[i] ] =  m_grid.getCellAABB(m_nbRanks[i]) ;
        }
};


template<typename TDynamicsSystem, typename TRankId>
bool ProcessTopologyGrid<TDynamicsSystem,TRankId>::belongsPointToProcess(const Vector3 & point, unsigned int &neighbourProcessRank) const {
    //TODO
    neighbourProcessRank = m_grid.getCellNumber(point);
    if(neighbourProcessRank == m_rank) {
        return true;
    }
    return false;
};


template<typename TDynamicsSystem, typename TRankId>
bool ProcessTopologyGrid<TDynamicsSystem,TRankId>::checkOverlap(const  RigidBodyType * body,
                                                                std::vector<RankIdType> & neighbourProcessRanks) const {

    typename RankToAABBType::iterator it;
    neighbourProcessRanks.clear();
    for(it= m_nbAABB.begin(); it != m_nbAABB.end(); it++) {
        if( m_Collider.checkOverlap(body,it->second) ) {
            neighbourProcessRanks.push_back(it->first);
        }
    }
    return neighbourProcessRanks.size() > 0;
};

}; //MPILayer

#endif
