// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_inclusion_InclusionSolverCONoG_hpp
#define GRSF_dynamics_inclusion_InclusionSolverCONoG_hpp

#include <fstream>
#include <iostream>
#include <memory>

#include "GRSF/common/Asserts.hpp"
#include "GRSF/common/LogDefines.hpp"
#include "GRSF/common/TypeDefs.hpp"

#include CollisionSolver_INCLUDE_FILE
#include "GRSF/dynamics/inclusion/PercussionPool.hpp"

#include InclusionSolverSettings_INCLUDE_FILE
#include "GRSF/common/SimpleLogger.hpp"
#include "GRSF/dynamics/inclusion/ContactGraph.hpp"

#if HAVE_CUDA_SUPPORT == 1
// include the  JOR Prox Velocity GPU module
#include "JORProxVelocityGPUModule.hpp"
#endif

#include "GRSF/common/CPUTimer.hpp"

/**
* @ingroup Inclusion
* @brief The inclusion solver for an ordered problem. Does not assemble G. Iterates over Contact Graph in a SOR fashion!
*/

class InclusionSolverCONoG
{
public:
    DEFINE_INCLUSIONS_SOLVER_CONFIG_TYPES

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    InclusionSolverCONoG(std::shared_ptr<CollisionSolverType> pCollisionSolver,
                         std::shared_ptr<DynamicsSystemType> pDynSys);
    ~InclusionSolverCONoG();

    void initializeLog(Logging::Log* pSolverLog, boost::filesystem::path folder_path);
    void reset();
    void resetForNextTimestep(unsigned int timeStepCounter = 0);  // Is called each iteration in the timestepper, so
                                                                  // that the InclusionSolver is able to reset matrices
                                                                  // which are dynamically added to during the
                                                                  // iteration! (like, h term)
    void solveInclusionProblem();

    std::string getIterationStats();
    std::string getStatsHeader();

    InclusionSolverSettingsType m_settings;

    unsigned int getNContacts()
    {
        return m_nContacts;
    }

protected:
    unsigned int m_globalIterationCounter;
    bool m_bConverged;
    PREC m_maxResidual;
    unsigned int m_isFinite;
    unsigned int m_nContacts;
    bool m_bUsedGPU;
    double m_timeProx, m_proxIterationTime;

    std::shared_ptr<CollisionSolverType> m_pCollisionSolver;
    std::shared_ptr<DynamicsSystemType> m_pDynSys;

    typename DynamicsSystemType::RigidBodySimContainerType& m_simBodies;
    typename DynamicsSystemType::RigidBodyStaticContainerType& m_staticBodies;

    // General CPU Iteration visitors (only SOR Prox on velocity level)
    using ContactGraphType = ContactGraph;
    ContactGraphType m_contactGraph;
    void initContactGraphForIteration(PREC alpha);

    // Different visitors for the various SOR implementations
    // For SOR_FULL, SOR_CONTACT
    ContactSorProxStepNodeVisitor<ContactGraphType>* m_pContactSorProxStepNodeVisitor = nullptr;
    FullSorProxStepNodeVisitor<ContactGraphType>* m_pFullSorProxStepNodeVisitor       = nullptr;
    // For SOR_NORMAL_TANGENTIAL
    NormalSorProxStepNodeVisitor<ContactGraphType>* m_pNormalSorProxStepNodeVisitor         = nullptr;
    TangentialSorProxStepNodeVisitor<ContactGraphType>* m_pTangentialSorProxStepNodeVisitor = nullptr;
    // Init Visitor for the contacts
    SorProxInitNodeVisitor<ContactGraphType>* m_pSorProxInitNodeVisitor = nullptr;

    void doSORProxCPU();
    inline void sorProxOverAllNodes();

#if HAVE_CUDA_SUPPORT == 1
    // Jor Prox Velocity GPU Module
    using JorProxGPUModuleType = JorProxVelocityGPUModule;
    JorProxGPUModuleType m_jorProxGPUModule;
#endif
    void doJORProxGPU();

    template <bool onlyNotInContactGraph = false>
    void integrateAllBodyVelocities();

    inline void doJorProx();
    inline void doSorProx();

    // Percussion cache
    PercussionPool* m_percussionPool                                        = nullptr;
    CachePercussionNodeVisitor<ContactGraphType>* m_pCachePercussionVisitor = nullptr;

    // Drift Corrector
    // DriftCorrector

    // Log
    Logging::Log* m_pSolverLog     = nullptr;
    Logging::Log* m_pSimulationLog = nullptr;

    // Residual File
    std::fstream m_iterationDataFile;
};

#endif
