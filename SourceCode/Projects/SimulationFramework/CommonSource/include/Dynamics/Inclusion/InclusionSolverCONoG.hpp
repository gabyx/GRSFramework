#ifndef InclusionSolverNTContactOrderedNoG_hpp
#define InclusionSolverNTContactOrderedNoG_hpp


#include <iostream>
#include <fstream>
#include <boost/shared_ptr.hpp>

#include "ConfigureFile.hpp"

#include "TypeDefs.hpp"
#include "LogDefines.hpp"
#include "AssertionDebug.hpp"

#include CollisionSolver_INCLUDE_FILE
#include "PercussionPool.hpp"

#include InclusionSolverSettings_INCLUDE_FILE
#include "ContactGraph.hpp"
#include "SimpleLogger.hpp"


#if HAVE_CUDA_SUPPORT == 1
#include "JorProxGPUVariant.hpp"
#include "SorProxGPUVariant.hpp"
#endif

#include <boost/timer/timer.hpp>



/**
* @ingroup Inclusion
* @brief The inclusion solver for an ordered problem. Does not assemble G. Iterates over Contact Graph in a SOR fashion!
*/

class InclusionSolverCONoG {
public:

    DEFINE_INCLUSIONS_SOLVER_CONFIG_TYPES

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    InclusionSolverCONoG(boost::shared_ptr<CollisionSolverType >  pCollisionSolver, boost::shared_ptr<DynamicsSystemType> pDynSys);
    ~InclusionSolverCONoG();

    void initializeLog( Logging::Log* pSolverLog, boost::filesystem::path folder_path );
    void reset();
    void resetForNextIter(); // Is called each iteration in the timestepper, so that the InclusionSolver is able to reset matrices which are dynamically added to during the iteration! (like, h term)
    void solveInclusionProblem();


    std::string getIterationStats();
    std::string getStatsHeader();

    unsigned int m_globalIterationCounter;
    bool m_bConverged;
    unsigned int m_isFinite;
    unsigned int m_nContacts;
    bool m_bUsedGPU;
    double m_timeProx, m_proxIterationTime;



    PercussionPool m_PercussionPool;

//    void reservePercussionPoolSpace(unsigned int nExpectedContacts);
//    void readFromPercussionPool(unsigned int index, const CollisionData * pCollData, VectorDyn & P_old);
//    void updatePercussionPool(const VectorDyn & P_old ) ;

    InclusionSolverSettingsType m_Settings;


protected:


    unsigned int m_nExpectedContacts;

    boost::shared_ptr<CollisionSolverType> m_pCollisionSolver;
    boost::shared_ptr<DynamicsSystemType>  m_pDynSys;

    typename DynamicsSystemType::RigidBodySimContainerType & m_SimBodies;
    typename DynamicsSystemType::RigidBodyStaticContainer & m_Bodies;

    typedef ContactGraph<ContactGraphMode::ForIteration> ContactGraphType;
    ContactGraphType m_ContactGraph;

    void integrateAllBodyVelocities();
    void initContactGraphForIteration(PREC alpha);

    inline void doJorProx();

    inline void doSorProx();
    inline void sorProxOverAllNodes();
    SorProxStepNodeVisitor * m_pSorProxStepNodeVisitor;
    SorProxInitNodeVisitor * m_pSorProxInitNodeVisitor;

    // Log
    Logging::Log *m_pSolverLog, *m_pSimulationLog;
    std::stringstream logstream;
};


#endif
