#ifndef InclusionSolverNTContactOrdered_hpp
#define InclusionSolverNTContactOrdered_hpp

#include <iostream>
#include <fstream>
#include <boost/shared_ptr.hpp>

#include "AssertionDebug.hpp"
#include "TypeDefs.hpp"
#include "LogDefines.hpp"
#include "ConfigureFile.hpp"

#include "CollisionSolver.hpp"
#include InclusionSolverSettings_INCLUDE_FILE
#include "ContactGraph.hpp"

#include "SimpleLogger.hpp"







/**
* @ingroup Inclusion
* @brief The inclusion solver for an ordered problem.
*/

class InclusionSolverCO {
public:

    DEFINE_INCLUSIONS_SOLVER_CONFIG_TYPES
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW




    InclusionSolverCO(boost::shared_ptr<CollisionSolverType >  pCollisionSolver, boost::shared_ptr<DynamicsSystemType> pDynSys);

    void initializeLog( Logging::Log* pSolverLog, boost::filesystem::path folder_path );
    void reset();
    void resetForNextIter(); // Is called each iteration in the timestepper, so that the InclusionSolver is able to reset matrices which are dynamically added to during the iteration! (like, h term)
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


    InclusionSolverSettingsType m_Settings;

    unsigned int getNContacts(){return m_nContacts;}

protected:

    static const unsigned int NDOFFriction;
    static const unsigned int ContactDim;

    unsigned int m_nExpectedContacts;

    boost::shared_ptr<CollisionSolverType> m_pCollisionSolver;
    boost::shared_ptr<DynamicsSystemType>  m_pDynSys;
    typename DynamicsSystemType::RigidBodySimContainerType & m_SimBodies;
    typename DynamicsSystemType::RigidBodyStaticContainer & m_Bodies;

    typedef ContactGraph<ContactGraphMode::NoIteration> ContactGraphType;
    ContactGraphType m_ContactGraph;

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

    MatrixDyn m_T;

    VectorDyn m_d;

    VectorDyn m_R;
    // ==========================================================================

    inline void setupRMatrix(PREC alpha);

#if HAVE_CUDA_SUPPORT == 1
    JorProxGPUVariant< JorProxGPUVariantSettingsWrapper<PREC,5,ConvexSets::RPlusAndDisk,true,300,true,10,false, TemplateHelper::Default>, ConvexSets::RPlusAndDisk > m_jorGPUVariant;
    //SorProxGPUVariant< SorProxGPUVariantSettingsWrapper<PREC,5,ConvexSets::RPlusAndDisk,true,300,true,10,true, RelaxedSorProxKernelSettings<32,ConvexSets::RPlusAndDisk> >,  ConvexSets::RPlusAndDisk > m_sorGPUVariant;
    SorProxGPUVariant< SorProxGPUVariantSettingsWrapper<PREC,1,ConvexSets::RPlusAndDisk,true,300,true,10,true,  TemplateHelper::Default >,  ConvexSets::RPlusAndDisk > m_sorGPUVariant;
#endif

    inline void doJorProx();
    inline void doSorProx();

    // Log
    Logging::Log *m_pSolverLog, *m_pSimulationLog;
    std::stringstream logstream;
};


#endif
