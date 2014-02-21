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
//#include "PercussionPool.hpp"

#include InclusionSolverSettings_INCLUDE_FILE

#include "SimpleLogger.hpp"

#include "MPICommunication.hpp"

#include "BodyCommunicator.hpp"

#include "InclusionCommunicator.hpp"
// those two include each other (forwarding)
#include "ContactGraphMPI.hpp"

#include "ContactGraphVisitorsMPI.hpp"

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
                         boost::shared_ptr< CollisionSolverType >  pCollisionSolver,
                         boost::shared_ptr< DynamicsSystemType> pDynSys,
                         boost::shared_ptr< ProcessCommunicatorType > pProcCom);
    ~InclusionSolverCONoG();

    void initializeLog( Logging::Log* pSolverLog, boost::filesystem::path folder_path );
    void reset();
    void resetForNextIter(); // Is called each iteration in the timestepper, so that the InclusionSolver is able to reset matrices which are dynamically added to during the iteration! (like, h term)
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


    InclusionSolverSettingsType m_Settings;


protected:

    //MPI Stuff
    boost::shared_ptr< ProcessCommunicatorType > m_pProcComm;
    boost::shared_ptr< ProcessInfoType > m_pProcInfo;
    const typename ProcessTopologyType::NeighbourRanksListType & m_nbRanks;

    unsigned int m_nExpectedContacts;

    boost::shared_ptr<CollisionSolverType> m_pCollisionSolver;
    boost::shared_ptr<DynamicsSystemType>  m_pDynSys;

    boost::shared_ptr<BodyCommunicator>  m_pBodyComm;


    /** Circulare template dependency of InclusionCommunicator and ContactGraph
    *   Can be solved with this combo trait class :-)
    *    struct ComboIncGraph {
    *        typedef InclusionCommunicator<ComboIncGraph> InclusionCommunicatorType;
    *        typedef ContactGraph<ComboIncGraph> ContactGraphType;
    *    };
    *    Nevertheless, we avoided this here
    */

    struct Combo{
        typedef InclusionCommunicator<Combo> InclusionCommunicatorType;
        typedef ContactGraph<Combo> ContactGraphType;
    };


    typedef Combo::InclusionCommunicatorType InclusionCommunicatorType;
    boost::shared_ptr<InclusionCommunicatorType> m_pInclusionComm;

    typedef Combo::ContactGraphType ContactGraphType;
    boost::shared_ptr<ContactGraphType> m_pContactGraph;
    /// =========================================================================




    typename DynamicsSystemType::RigidBodySimContainerType & m_SimBodies;
    typename DynamicsSystemType::RigidBodyStaticContainer & m_Bodies;

    typename ContactGraphType::SplitBodyNodeDataListType m_nodesSplitBody;
    typename ContactGraphType::NodeListType              m_nodesLocal, m_nodesRemote;



    void integrateAllBodyVelocities();
    void initContactGraphForIteration(PREC alpha);

    inline void doJorProx();

    inline void doSorProx();
    inline void sorProxOverAllNodes();
    inline void finalizeSorProx();

    SorProxStepNodeVisitor<ContactGraphType> * m_pSorProxStepNodeVisitor;
    SorProxInitNodeVisitor<ContactGraphType> * m_pSorProxInitNodeVisitor;
    SorProxStepSplitNodeVisitor<ContactGraphType> * m_pSorProxStepSplitNodeVisitor;

    // Log
    Logging::Log *m_pSolverLog, *m_pSimulationLog;
    std::stringstream logstream;




};


#endif
