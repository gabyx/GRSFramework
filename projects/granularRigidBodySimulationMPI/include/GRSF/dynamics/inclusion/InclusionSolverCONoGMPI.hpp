// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_inclusion_InclusionSolverCONoGMPI_hpp
#define GRSF_dynamics_inclusion_InclusionSolverCONoGMPI_hpp

#include <fstream>
#include <iostream>

#include <memory>

#include "GRSF/common/Asserts.hpp"

#include "GRSF/common/LogDefines.hpp"
#include "GRSF/common/TypeDefs.hpp"

#include CollisionSolver_INCLUDE_FILE
//#include "GRSF/dynamics/inclusion/PercussionPool.hpp"

#include InclusionSolverSettings_INCLUDE_FILE

#include "GRSF/common/SimpleLogger.hpp"

#include "GRSF/dynamics/general/MPICommunication.hpp"

#include "GRSF/dynamics/general/BodyCommunicator.hpp"

#include "GRSF/dynamics/inclusion/InclusionCommunicator.hpp"
// those two include each other (forwarding)
#include "GRSF/dynamics/inclusion/ContactGraphMPI.hpp"

#include "GRSF/dynamics/inclusion/ContactGraphVisitorsMPI.hpp"

#include "GRSF/common/CPUTimer.hpp"

/**
* @ingroup Inclusion
* @brief The inclusion solver for an ordered problem. Does not assemble G. Iterates over Contact Graph in a SOR fashion!
*/

class InclusionSolverCONoGMPI
{
public:
    DEFINE_INCLUSIONS_SOLVER_CONFIG_TYPES

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using ProcessCommunicatorType = MPILayer::ProcessCommunicator;
    using ProcessInfoType         = typename ProcessCommunicatorType::ProcessInfoType;
    using ProcessTopologyType     = typename ProcessCommunicatorType::ProcessInfoType::ProcessTopologyType;

    InclusionSolverCONoGMPI(std::shared_ptr<BodyCommunicator> pBodyComm,
                            std::shared_ptr<CollisionSolverType> pCollisionSolver,
                            std::shared_ptr<DynamicsSystemType> pDynSys,
                            std::shared_ptr<ProcessCommunicatorType> pProcComm);
    ~InclusionSolverCONoGMPI();

    void initializeLog(Logging::Log* pSolverLog, boost::filesystem::path folder_path);
    void reset();
    void resetTopology();
    void resetForNextTimestep();  // Is called each iteration in the timestepper, so that the InclusionSolver is able to
                                  // reset matrices which are dynamically added to during the iteration! (like, h term)
    void solveInclusionProblem(PREC currentSimulationTime);

    std::string getIterationStats();
    std::string getStatsHeader();
    PREC m_currentSimulationTime;
    unsigned int m_globalIterationCounter;
    bool m_bConverged;
    unsigned int m_isFinite;
    unsigned int m_nContacts;
    unsigned int m_nLocalNodes;
    unsigned int m_nRemoteNodes;
    unsigned int m_nSplitBodyNodes;

    bool m_bUsedGPU;
    double m_timeProx, m_proxIterationTime;

    InclusionSolverSettingsType m_settings;

protected:
    // MPI Stuff

    //    const typename ProcessTopologyType::NeighbourRanksListType & m_nbRanks;

    std::shared_ptr<DynamicsSystemType> m_pDynSys;
    std::shared_ptr<CollisionSolverType> m_pCollisionSolver;

    std::shared_ptr<BodyCommunicator> m_pBodyComm;

    std::shared_ptr<ProcessCommunicatorType> m_pProcComm;
    // std::shared_ptr< ProcessInfoType > m_pProcInfo;

    /** Circulare template dependency of InclusionCommunicator and ContactGraph
    *   Can be solved with this combo trait class :-)
    *    struct ComboIncGraph {
    *        using InclusionCommunicatorType = InclusionCommunicator<ComboIncGraph>;
    *        using ContactGraphType = ContactGraph<ComboIncGraph>;
    *    };
    *    Nevertheless, we avoided this here
    */

    struct Combo
    {
        using InclusionCommunicatorType = InclusionCommunicator<Combo>;
        using ContactGraphType          = ContactGraph<Combo>;
    };

    using InclusionCommunicatorType             = Combo::InclusionCommunicatorType;
    InclusionCommunicatorType* m_pInclusionComm = nullptr;

    using ContactGraphType            = Combo::ContactGraphType;
    ContactGraphType* m_pContactGraph = nullptr;
    /// =========================================================================

    typename DynamicsSystemType::RigidBodySimContainerType& m_simBodies;
    typename DynamicsSystemType::RigidBodyStaticContainerType& m_staticBodies;

    void integrateAllBodyVelocities();
    void initContactGraphForIteration(PREC alpha);

    inline void doJorProx();

    inline void doSorProx();
    inline void sorProxOverAllNodes();
    inline void finalizeSorProx();

    // Different visitors for the various SOR implementations
    // For SOR_FULL, SOR_CONTACT
    ContactSorProxStepNodeVisitor<ContactGraphType>* m_pContactSorProxStepNodeVisitor = nullptr;
    FullSorProxStepNodeVisitor<ContactGraphType>* m_pFullSorProxStepNodeVisitor       = nullptr;
    // For SOR_NORMAL_TANGENTIAL
    NormalSorProxStepNodeVisitor<ContactGraphType>* m_pNormalSorProxStepNodeVisitor         = nullptr;
    TangentialSorProxStepNodeVisitor<ContactGraphType>* m_pTangentialSorProxStepNodeVisitor = nullptr;
    // Init Visitor for the contacts
    SorProxInitNodeVisitor<ContactGraphType>* m_pSorProxInitNodeVisitor = nullptr;

    SorProxStepSplitNodeVisitor<ContactGraphType>* m_pSorProxStepSplitNodeVisitor = nullptr;

    // Log
    Logging::Log* m_pSolverLog     = nullptr;
    Logging::Log* m_pSimulationLog = nullptr;

    std::stringstream logstream;

    PREC m_avgIterTimeProxLocalRemoteNodes;
    PREC m_avgIterTimeProxSplitNodes;
};

#endif
