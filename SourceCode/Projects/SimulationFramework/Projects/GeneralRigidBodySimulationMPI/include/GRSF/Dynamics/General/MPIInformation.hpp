#ifndef GRSF_Dynamics_General_MPIInformation_hpp
#define GRSF_Dynamics_General_MPIInformation_hpp

#include <mpi.h>

#include "GRSF/Common/AssertionDebug.hpp"
#include "GRSF/Common/TypeDefs.hpp"

#include "GRSF/Dynamics/General/CartesianGrid.hpp"

//#include "GRSF/Dynamics/Collision/Collider.hpp"
#include "GRSF/Singeltons/MPIGlobalCommunicators.hpp"
#include "GRSF/Dynamics/General/MPITopology.hpp"

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

    template<typename... T>
    void createProcTopoGrid(T &&... t){
        m_procTopo.createProcessTopologyGrid(m_rank, MPILayer::ProcessInformation::MASTER_RANK , std::forward<T>(t)...  );
    }

    template<typename... T>
    void createProcTopoKdTree(T &&... t){
        m_procTopo.createProcessTopologyKdTree(m_rank, MPILayer::ProcessInformation::MASTER_RANK , std::forward<T>(t)...  );
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

