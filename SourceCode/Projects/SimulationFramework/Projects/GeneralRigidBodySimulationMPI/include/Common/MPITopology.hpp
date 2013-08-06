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

    virtual bool belongsPointToProcess(const Vector3 & point, TRankId &neighbourProcessRank) const{
        ERRORMSG("The ProcessTopology::belongsPointToProcess has not been implemented!");
    }
    virtual bool belongsPointToProcess(const Vector3 & point) const{
        ERRORMSG("The ProcessTopology::belongsPointToProcess has not been implemented!");
    }

    virtual const std::vector<TRankId> & getNeigbourRanks() const{
        ERRORMSG("The ProcessTopology::belongsPointToProcess has not been implemented!");
    }

    virtual bool belongsBodyToProcess(const RigidBodyType * body) const{
        ERRORMSG("The ProcessTopology::belongsBodyToProcess has not been implemented!");
    }

    virtual bool belongsBodyToProcess(const RigidBodyType * body, TRankId &neighbourProcessRank) const{
        ERRORMSG("The ProcessTopology::belongsBodyToProcess has not been implemented!");
    }

    virtual bool checkOverlap(const  RigidBodyType * body, std::vector<TRankId> & neighbourProcessRanks) const{
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

    typedef TRankId RankIdType;

    ProcessTopologyGrid(  const Vector3 & minPoint,
                          const Vector3 & maxPoint,
                          const MyMatrix<unsigned int>::Vector3 & dim,
                          unsigned int processRank): m_grid(minPoint,maxPoint, dim, ProcessInformation<TDynamicsSystem>::MASTER_RANK ) {
        m_rank = processRank;
        //Initialize neighbours
        m_nbRanks = m_grid.getCellNeigbours(m_rank);

    }

    bool checkOverlap(const  RigidBodyType * body, std::vector<RankIdType> & neighbourProcessRanks) const{

    }

    bool belongsBodyToProcess(const RigidBodyType * body) const{
        unsigned int nb;
        return belongsPointToProcess(body->m_r_S,nb);
    }

    bool belongsBodyToProcess(const RigidBodyType * body, RankIdType &neighbourProcessRank) const{
        return belongsPointToProcess(body->m_r_S,neighbourProcessRank);
    }

    bool belongsPointToProcess(const Vector3 & point) const {
        unsigned int nb;
        return belongsPointToProcess(point,nb);
    }


    bool belongsPointToProcess(const Vector3 & point, unsigned int &neighbourProcessRank) const {
        //TODO
        neighbourProcessRank = m_grid.getCellNumber(point);
        if(neighbourProcessRank == m_rank) {
            return true;
        }
        return false;
    }

    const std::vector<unsigned int> & getNeigbourRanks() const{
        return m_nbRanks;
    }


private:
    unsigned int m_rank; ///< Own rank;
    std::vector<unsigned int> m_nbRanks; ///< Neighbour ranks
    CartesianGrid<LayoutConfigType,NoCellData> m_grid;

    Collider<DynamicsSystemType> m_Collider;
};

};

#endif
