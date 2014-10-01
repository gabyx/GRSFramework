#ifndef MPIInformation_hpp
#define MPIInformation_hpp

#include <mpi.h>

#include "AssertionDebug.hpp"
#include "TypeDefs.hpp"

#include "CartesianGrid.hpp"

//#include "Collider.hpp"
#include "MPIGlobalCommunicators.hpp"
#include "MPITopology.hpp"

namespace MPILayer {


class ProcessInformation {

public:

    using ProcessTopologyType = ProcessTopology;

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

    DEFINE_MPI_INFORMATION_CONFIG_TYPES

    ProcessInformation(MPI_Comm comm): m_comm(comm) {
        initialize();
        m_procTopo.init(m_rank);
    }

    ~ProcessInformation() {

    }

    inline RankIdType getMasterRank() const {
        return MASTER_RANK;
    };

    inline bool hasMasterRank() const{
        if(m_rank == MASTER_RANK){
            return true;
        }
        return false;
    }

    inline RankIdType getRank() const {
        return m_rank;
    };

    inline unsigned int getNProcesses() const {
        return m_nProcesses;
    };

    inline MPI_Comm getCommunicator() const{
        return m_comm;
    };

    /** Global name */
    inline std::string getName() const {
        return m_name;
    };

    inline void setName(std::string name) {
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
    const ProcessTopology* getProcTopo() const{
        return &m_procTopo;
    };

protected:

    static const int MASTER_RANK = 0;

    void initialize() {
        int v;

        // get world rank
        MPI_Comm comm = MPIGlobalCommunicators::getSingletonPtr()->getCommunicator(MPILayer::MPICommunicatorId::WORLD_COMM);
        MPI_Comm_rank(comm,&v);
        std::stringstream s;
        s << "SimProcess_"<<v;
        m_name = s.str();

        //Get specific rank and size for this comm
        MPI_Comm_rank(m_comm,&v);
        m_rank = v;
        MPI_Comm_size(m_comm,&v);
        m_nProcesses = v;

    };


    MPI_Comm m_comm; ///< communicator

    ProcessTopology m_procTopo;

    RankIdType m_rank;          ///< rank of communicator m_comm
    unsigned int m_nProcesses;  ///< processes in m_comm
    std::string m_name;
};




};

#endif

