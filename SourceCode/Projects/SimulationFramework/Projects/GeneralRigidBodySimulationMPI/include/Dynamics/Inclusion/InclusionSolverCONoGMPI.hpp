#ifndef InclusionSolverNTContactOrderedNoG_hpp
#define InclusionSolverNTContactOrderedNoG_hpp


#include <iostream>
#include <fstream>


#include <boost/timer/timer.hpp>
#include <boost/shared_ptr.hpp>

#include "AssertionDebug.hpp"

#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include CollisionSolver_INCLUDE_FILE
#include "PercussionPool.hpp"

#include "InclusionSolverSettings.hpp"

#include "SimpleLogger.hpp"

#include "MPICommunication.hpp"

#include "BodyCommunicator.hpp"
#include "InclusionCommunicator.hpp"

#include "ContactGraphMPI.hpp"


/**
* @ingroup Inclusion
* @brief The inclusion solver for an ordered problem. Does not assemble G. Iterates over Contact Graph in a SOR fashion!
*/

class InclusionSolverCONoG {
public:

    DEFINE_INCLUSIONS_SOLVER_CONFIG_TYPES

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef typename MPILayer::ProcessCommunicator                                      ProcessCommunicatorType;
    typedef typename ProcessCommunicatorType::ProcessInfoType                           ProcessInfoType;
    typedef typename ProcessCommunicatorType::ProcessInfoType::ProcessTopologyType      ProcessTopologyType;

    InclusionSolverCONoG(boost::shared_ptr< BodyCommunicator >  pBodyComm,
                         boost::shared_ptr<CollisionSolverType >  pCollisionSolver,
                         boost::shared_ptr<DynamicsSystemType> pDynSys,
                         boost::shared_ptr< ProcessCommunicatorType > pProcCom);
    ~InclusionSolverCONoG();

    void initializeLog( Logging::Log* pSolverLog, boost::filesystem::path folder_path );
    void reset();
    void resetForNextIter(); // Is called each iteration in the timestepper, so that the InclusionSolver is able to reset matrices which are dynamically added to during the iteration! (like, h term)
    void solveInclusionProblem();


    std::string getIterationStats();
    unsigned int m_iterationsNeeded;
    bool m_bConverged;
    unsigned int m_isFinite;
    unsigned int m_nContacts;
    bool m_bUsedGPU;
    double m_timeProx, m_proxIterationTime;



    PercussionPool m_PercussionPool;

//    void reservePercussionPoolSpace(unsigned int nExpectedContacts);
//    void readFromPercussionPool(unsigned int index, const CollisionData * pCollData, VectorDyn & P_old);
//    void updatePercussionPool(const VectorDyn & P_old ) ;

    InclusionSolverSettings m_Settings;


protected:

    //MPI Stuff
    boost::shared_ptr< ProcessCommunicatorType > m_pProcComm;
    boost::shared_ptr< ProcessInfoType > m_pProcInfo;
    const typename ProcessTopologyType::NeighbourRanksListType & m_nbRanks;

    unsigned int m_nExpectedContacts;

    boost::shared_ptr<CollisionSolverType> m_pCollisionSolver;
    boost::shared_ptr<DynamicsSystemType>  m_pDynSys;

    boost::shared_ptr<BodyCommunicator>  m_pBodyComm;

    typedef InclusionCommunicator InclusionCommunicatorType;
    boost::shared_ptr<InclusionCommunicatorType> m_pInclusionComm;

     // Graph // needs m_nbDataMap
    typedef ContactGraph ContactGraphType;
    boost::shared_ptr<ContactGraphType> m_pContactGraph;

    typename DynamicsSystemType::RigidBodySimContainerType & m_SimBodies;
    typename DynamicsSystemType::RigidBodyNotAniContainer & m_Bodies;

    void integrateAllBodyVelocities();
    void initContactGraphForIteration(PREC alpha);

    inline void doJorProx();

    inline void doSorProx();
    inline void sorProxOverAllNodes();
    SorProxStepNodeVisitor<ContactGraphType> * m_pSorProxStepNodeVisitor;
    SorProxInitNodeVisitor<ContactGraphType> * m_pSorProxInitNodeVisitor;

    // Log
    Logging::Log *m_pSolverLog, *m_pSimulationLog;
    std::stringstream logstream;




};


#endif
