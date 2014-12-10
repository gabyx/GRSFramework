#ifndef GMSF_Dynamics_General_MPIInformation_hpp
#define GMSF_Dynamics_General_MPIInformation_hpp

#include <mpi.h>

#include "GMSF/Common/AssertionDebug.hpp"
#include "GMSF/Common/TypeDefs.hpp"

#include "GMSF/Dynamics/General/CartesianGrid.hpp"

//#include "GMSF/Dynamics/Collision/Collider.hpp"
#include "GMSF/Singeltons/MPIGlobalCommunicators.hpp"
#include "GMSF/Dynamics/General/MPITopology.hpp"

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

    void createProcTopoGrid(const AABB & aabb,
                            const MyMatrix<unsigned int>::Array3 & dim,
                            bool aligned = true,
                            const Matrix33 & A_IK = Matrix33::Identity()){

        m_procTopo.createProcessTopologyGrid(m_rank, MPILayer::ProcessInformation::MASTER_RANK ,
                                             aabb,dim, aligned, A_IK  );    }

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
        MPI_Comm comm = MPIGlobalCommunicators::getSingleton().getCommunicator(MPILayer::MPICommunicatorId::WORLD_COMM);
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

