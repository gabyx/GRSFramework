// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_general_MPIInformation_hpp
#define GRSF_dynamics_general_MPIInformation_hpp

#include <mpi.h>

#include "GRSF/common/Asserts.hpp"
#include "GRSF/common/TypeDefs.hpp"

#include "GRSF/dynamics/general/CartesianGrid.hpp"

//#include "GRSF/dynamics/collision/Collider.hpp"
#include "GRSF/dynamics/general/MPITopology.hpp"
#include "GRSF/singeltons/MPIGlobalCommunicators.hpp"

namespace MPILayer
{
class ProcessInformation
{
public:
    using ProcessTopologyType = ProcessTopology;

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

    DEFINE_MPI_INFORMATION_CONFIG_TYPES

    ProcessInformation(MPI_Comm comm) : m_comm(comm)
    {
        initialize();
        m_procTopo.init(m_rank);
    }

    ~ProcessInformation()
    {
    }

    inline RankIdType getMasterRank() const
    {
        return MASTER_RANK;
    };

    inline bool hasMasterRank() const
    {
        if (m_rank == MASTER_RANK)
        {
            return true;
        }
        return false;
    }

    inline RankIdType getRank() const
    {
        return m_rank;
    };

    inline unsigned int getNProcesses() const
    {
        return m_nProcesses;
    };

    inline MPI_Comm getCommunicator() const
    {
        return m_comm;
    };

    /** Global name */
    inline std::string getName() const
    {
        return m_name;
    };

    inline void setName(std::string name)
    {
        m_name = name;
    };

    template <typename... T>
    void createProcTopoGrid(T&&... t)
    {
        m_procTopo.createProcessTopologyGrid(MPILayer::ProcessInformation::MASTER_RANK, std::forward<T>(t)...);
    }

    template <typename... T>
    void createProcTopoKdTree(T&&... t)
    {
        m_procTopo.createProcessTopologyKdTree(MPILayer::ProcessInformation::MASTER_RANK, std::forward<T>(t)...);
    }

    ProcessTopology* getProcTopo()
    {
        return &m_procTopo;
    };
    const ProcessTopology* getProcTopo() const
    {
        return &m_procTopo;
    };

protected:
    static const int MASTER_RANK = 0;

    void initialize()
    {
        int v;

        // get world rank
        MPI_Comm comm = MPIGlobalCommunicators::getSingleton().getCommunicator(MPILayer::MPICommunicatorId::WORLD_COMM);
        MPI_Comm_rank(comm, &v);
        std::stringstream s;
        s << "SimProcess_" << v;
        m_name = s.str();

        // Get specific rank and size for this comm
        MPI_Comm_rank(m_comm, &v);
        m_rank = v;
        MPI_Comm_size(m_comm, &v);
        m_nProcesses = v;
    };

    MPI_Comm m_comm;  ///< communicator

    ProcessTopology m_procTopo;

    RankIdType   m_rank;        ///< rank of communicator m_comm
    unsigned int m_nProcesses;  ///< processes in m_comm
    std::string  m_name;
};
};

#endif
