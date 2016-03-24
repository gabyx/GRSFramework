#ifndef GRSF_dynamics_inclusion_InclusionSolverCO_hpp
#define GRSF_dynamics_inclusion_InclusionSolverCO_hpp

#include <iostream>
#include <fstream>
#include <memory>

#include "GRSF/common/AssertionDebug.hpp"
#include "GRSF/common/TypeDefs.hpp"
#include "GRSF/common/LogDefines.hpp"

#include "GRSF/ConfigFiles/ConfigureFile.hpp"

#include "GRSF/dynamics/collision/CollisionSolver.hpp"
#include InclusionSolverSettings_INCLUDE_FILE
#include "GRSF/dynamics/inclusion/ContactGraph.hpp"

#include "GRSF/common/SimpleLogger.hpp"



#if HAVE_CUDA_SUPPORT == 1
#include "JorProxGPUVariant.hpp"
#include "SorProxGPUVariant.hpp"
#endif



/**
* @ingroup Inclusion
* @brief The inclusion solver for an ordered problem.
*/

class InclusionSolverCO {
public:

    DEFINE_INCLUSIONS_SOLVER_CONFIG_TYPES
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW




    InclusionSolverCO(std::shared_ptr<CollisionSolverType >  pCollisionSolver, std::shared_ptr<DynamicsSystemType> pDynSys);

    void initializeLog( Logging::Log* pSolverLog, boost::filesystem::path folder_path );
    void reset();
    void resetForNextTimestep(); // Is called each iteration in the timestepper, so that the InclusionSolver is able to reset matrices which are dynamically added to during the iteration! (like, h term)
    void solveInclusionProblem();

    std::string getIterationStats();
    std::string getStatsHeader();
    PREC m_G_conditionNumber;
    PREC m_G_notDiagDominant;
    unsigned int m_globalIterationCounter;
    bool m_bConverged;
    unsigned int m_isFinite;
    unsigned int m_nContacts;
    bool m_bUsedGPU;
    double m_timeProx, m_proxIterationTime;


    InclusionSolverSettingsType m_settings;

    unsigned int getNContacts(){return m_nContacts;}

protected:

    static const unsigned int NDOFFriction;
    static const unsigned int ContactDim;

    unsigned int m_nExpectedContacts;

    std::shared_ptr<CollisionSolverType> m_pCollisionSolver;
    std::shared_ptr<DynamicsSystemType>  m_pDynSys;
    typename DynamicsSystemType::RigidBodySimContainerType & m_simBodies;
    typename DynamicsSystemType::RigidBodyStaticContainerType & m_staticBodies;

    using ContactGraphType = ContactGraph;
    ContactGraphType m_contactGraph;

    // Matrices for solving the inclusion ===========================
    PREC m_nLambdas;

    VectorDyn m_mu;

    // Two buffers to exchange fastly the percussions in the prox iterations
    VectorDyn* m_P_front;
    VectorDyn* m_P_back;

#define P_back (*m_P_back)
#define P_front (*m_P_front)

    void swapPercussionBuffer();
    void resetPercussionBuffer();
    VectorDyn m_P_1;
    VectorDyn m_P_2;
    // ==========================

    MatrixDynDyn m_T;

    VectorDyn m_d;

    VectorDyn m_R;
    // ==========================================================================

    inline void setupRMatrix(PREC alpha);

#if HAVE_CUDA_SUPPORT == 1
    JorProxGPUVariant< JorProxGPUVariantSettingsWrapper<PREC,5,ConvexSets::RPlusAndDisk,true,300,true,10,false, TypeTraitsHelper::Default>, ConvexSets::RPlusAndDisk > m_jorGPUVariant;
    //SorProxGPUVariant< SorProxGPUVariantSettingsWrapper<PREC,5,ConvexSets::RPlusAndDisk,true,300,true,10,true, RelaxedSorProxKernelSettings<32,ConvexSets::RPlusAndDisk> >,  ConvexSets::RPlusAndDisk > m_sorGPUVariant;
    SorProxGPUVariant< SorProxGPUVariantSettingsWrapper<PREC,1,ConvexSets::RPlusAndDisk,true,300,true,10,true,  TypeTraitsHelper::Default >,  ConvexSets::RPlusAndDisk > m_sorGPUVariant;
#endif

    inline void doJorProx();
    inline void doSorProx();

    // Log
    Logging::Log *m_pSolverLog, *m_pSimulationLog;
    std::stringstream logstream;
};


#endif
