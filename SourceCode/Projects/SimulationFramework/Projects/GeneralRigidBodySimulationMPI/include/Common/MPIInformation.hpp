#ifndef MPIInformation_hpp
#define MPIInformation_hpp

#include <mpi.h>

#include "AssertionDebug.hpp"
#include "TypeDefs.hpp"

#include "CartesianGrid.hpp"

//#include "Collider.hpp"

#include "MPITopology.hpp"

namespace MPILayer {


class ProcessInformation {

public:

    typedef ProcessTopology ProcessTopologyType;

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

    DEFINE_MPI_INFORMATION_CONFIG_TYPES

    static const int MASTER_RANK = 0;

    ProcessInformation() {
        initialize();
        m_procTopo.init(m_rank);
    }

    ~ProcessInformation() {

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

    MPI_Comm getMPIComm(){
        return MPI_COMM_WORLD;
    }

    std::string getName() const {
        return m_name;
    };
    void setName(std::string name) {
        m_name = name;
    };

    void createProcTopoGrid(const Vector3 & minPoint,
                            const Vector3 & maxPoint,
                            const MyMatrix<unsigned int>::Vector3 & dim){
        m_procTopo.createProcessTopologyGrid(minPoint,maxPoint,dim, m_rank, MPILayer::ProcessInformation::MASTER_RANK  );
    }

    ProcessTopology* getProcTopo(){
        return &m_procTopo;
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

    ProcessTopology m_procTopo;

    RankIdType m_rank;
    int m_nProcesses;
    std::string m_name;
};




};

#endif

