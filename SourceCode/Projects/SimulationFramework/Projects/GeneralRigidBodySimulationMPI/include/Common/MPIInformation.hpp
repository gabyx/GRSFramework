#ifndef MPIInformation_hpp
#define MPIInformation_hpp

#include <mpi.h>

#include "AssertionDebug.hpp"
#include "TypeDefs.hpp"

#include "CartesianGrid.hpp"

//#include "Collider.hpp"

#include "MPITopology.hpp"

namespace MPILayer {

template<typename TDynamicsSystem>
class ProcessInformation {

public:

    typedef typename TDynamicsSystem::DynamicsSystemConfig DynamicsSystemConfig;
    typedef TDynamicsSystem DynamicsSystemType;
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES_OF(DynamicsSystemConfig)

    typedef unsigned int RankIdType;
    typedef typename MPILayer::ProcessTopology<DynamicsSystemType,RankIdType> ProcessTopologyType;

    static const int MASTER_RANK = 0;

    ProcessInformation() {
        m_pProcTopo = NULL;
        initialize();
    }

    ~ProcessInformation() {
        if(m_pProcTopo) {
            delete m_pProcTopo;
        }
    }

    RankIdType getMasterRank() const {
        return MASTER_RANK;
    };

    bool hasMasterRank() const{
        if(m_rank == MASTER_RANK){
            return true;
        }
        return false;
    }

    RankIdType getRank() const {
        return m_rank;
    };
    void setRank(unsigned int rank) {
        m_rank = rank;
    };

    unsigned int getNProcesses() const {
        return m_nProcesses;
    };

    std::string getName() const {
        return m_name;
    };
    void setName(std::string name) {
        m_name = name;
    };

    void createProcTopoGrid(const Vector3 & minPoint,
                            const Vector3 & maxPoint,
                            const MyMatrix<unsigned int>::Vector3 & dim){
        if(m_pProcTopo){
            delete m_pProcTopo;
        }
        m_pProcTopo = new MPILayer::ProcessTopologyGrid<DynamicsSystemType,RankIdType>(minPoint,maxPoint,dim, getRank() );
    }

    const ProcessTopologyType * getProcTopo() const{
        ASSERTMSG(m_pProcTopo,"m_pProcTopo == NULL");
        return m_pProcTopo;
    };

private:

    void initialize() {
        int rank;
        MPI_Comm_rank(MPI_COMM_WORLD,&rank);
        m_rank = rank;
        MPI_Comm_size(MPI_COMM_WORLD,&this->m_nProcesses);

        std::stringstream s;
        s << "Process_"<<m_rank;
        m_name = s.str();
    };

    ProcessTopologyType * m_pProcTopo;

    RankIdType m_rank;
    int m_nProcesses;
    std::string m_name;
};




};

#endif

