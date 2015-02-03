#ifndef GRSF_Dynamics_General_MPITopologyBuilder_hpp
#define GRSF_Dynamics_General_MPITopologyBuilder_hpp

#include <mpi.h>

#include <pugixml.hpp>

#include <functional>
#include <numeric>

#include "GRSF/Common/AssertionDebug.hpp"
#include "GRSF/Common/TypeDefs.hpp"
#include "GRSF/Common/LogDefines.hpp"

#include "GRSF/Singeltons/FileManager.hpp"

#include "GRSF/Dynamics/General/MPIMessages.hpp"
#include "GRSF/Dynamics/General/MPICommunication.hpp"

#include "GRSF/Systems/SceneParserMPI.hpp"
#include "GRSF/Systems/SceneParserModulesCreatorTB.hpp"

#include DynamicsSystem_INCLUDE_FILE
#include "GRSF/Dynamics/Collision/Collider.hpp"

#include "GRSF/Dynamics/General/MPITopologyBuilderSettings.hpp"

#include "GRSF/Dynamics/Collision/Geometry/AABB.hpp"
#include "GRSF/Dynamics/Collision/Geometry/OOBB.hpp"
#include "ApproxMVBB/ComputeApproxMVBB.hpp"

namespace MPILayer {


class TopologyBuilder {

public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
    DEFINE_MPI_INFORMATION_CONFIG_TYPES

    using TopologyBuilderEnumType = TopologyBuilderEnum;
    using ProcessCommunicatorType = MPILayer::ProcessCommunicator;

    using RebuildSettings = TopologyBuilderSettings::RebuildSettings;
    using MassPointPredSettings = TopologyBuilderSettings::MassPointPredSettings;


    TopologyBuilder(std::shared_ptr<DynamicsSystemType> pDynSys,
                    std::shared_ptr<ProcessCommunicatorType > pProcCommunicator,
                    RebuildSettings rebuildSettings, MassPointPredSettings massPointPredSettings ):
        m_pDynSys(pDynSys), m_pProcCommunicator(pProcCommunicator), m_rebuildSettings(rebuildSettings), m_massPointPredSettings(massPointPredSettings) {

        m_pSimulationLog = nullptr;
        m_pSimulationLog = Logging::LogManager::getSingleton().getLog("SimulationLog");
        ASSERTMSG(m_pSimulationLog, "There is no SimulationLog in the LogManager!");
    }

    PREC m_currentTime;
    unsigned int m_builtTopologies = 0;

    bool checkAndRebuild(unsigned int timeStep, PREC currentTime){
        // Check each X timestep
        if( timeStep!= 0 && timeStep % m_rebuildSettings.m_policyCheckEachXTimeStep == 0){

            if(m_rebuildSettings.m_policy == RebuildSettings::Policy::ALWAYS_REBUILD){

                this->rebuildTopology(currentTime);
                return true;

            }else if(m_rebuildSettings.m_policy == RebuildSettings::Policy::BODY_LIMIT){

                bool rebuild = false;
                if( m_pDynSys->m_simBodies.size() >= m_rebuildSettings.m_bodyLimit){
                    rebuild = true;
                }
                // Agree on all processes to rebuild!
                rebuild = communicateRebuild(rebuild);

                if(rebuild){
                    this->rebuildTopology(currentTime);
                    return true;
                }

            }
        }
        return false;
    };

    void initLogs(boost::filesystem::path localSimFolderPath){
        m_localSimFolderPath = localSimFolderPath;
    }

    virtual void initTopology(PREC currentTime = 0) = 0;
    virtual void rebuildTopology(PREC currentTime) = 0;

    virtual ~TopologyBuilder() {}

protected:

    bool communicateRebuild(bool rebuild){
         // Synchronize rebuilding over all processes by communication
        LOGTBLEVEL1(m_pSimulationLog,"MPI> Gather topology rebuild flags: "<<std::endl;)
        std::vector<char> rebuildGather;
        char r = rebuild? 1 : 0;
        m_pProcCommunicator->allGather( r ,rebuildGather, MPILayer::MPICommunicatorId::SIM_COMM);

        if( TOPOBUILDER_LOGLEVEL > 2){
            Utilities::printVectorNoCopy(*m_pSimulationLog,rebuildGather.begin(),rebuildGather.end(),",");
        }

        // Sum up all boolean values and decide if all neighbours rebuild?
        unsigned int n = std::accumulate(rebuildGather.begin(), rebuildGather.end() , 0, std::plus<char>() );
        LOGTBLEVEL1(m_pSimulationLog,"MPI> n="<<n<<" processes want to rebuild!"<<std::endl;)
        if(n>=0){ // if one or more process wants to rebuild then rebuild!
            LOGTBLEVEL1(m_pSimulationLog,"MPI> Rebuild accepted!"<<std::endl;)
            return true;
        }
        LOGTBLEVEL1(m_pSimulationLog,"MPI> Rebuild not accepted!"<<std::endl;)
        return false;
    }

    std::shared_ptr<DynamicsSystemType>  m_pDynSys;
    std::shared_ptr<ProcessCommunicatorType> m_pProcCommunicator;


    TopologyBuilderEnumType m_type;
    RebuildSettings m_rebuildSettings;
    MassPointPredSettings m_massPointPredSettings;

    Logging::Log * m_pSimulationLog;
    boost::filesystem::path m_localSimFolderPath;
};


template<typename TProcCommunicator>
class GridTopologyBuilder : public TopologyBuilder {

    friend class ParserModulesCreatorTopoBuilder<GridTopologyBuilder>;

    friend class TopologyBuilderMessageWrapperBodies<GridTopologyBuilder>;
    friend class TopologyBuilderMessageWrapperResults<GridTopologyBuilder>;
    friend class TopologyBuilderMessageWrapperOrientation<GridTopologyBuilder>;

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
    DEFINE_MPI_INFORMATION_CONFIG_TYPES

    using ProcessCommunicatorType = TProcCommunicator;

    using RigidBodyStatesContainerType = typename DynamicsSystemType::RigidBodyStatesContainerType;
    using RigidBodySimContainerType = typename DynamicsSystemType::RigidBodySimContainerType ;
    using RigidBodyStaticContainerType = typename DynamicsSystemType::RigidBodyStaticContainerType ;


private:

    template< typename > friend struct ParserModulesCreatorTopoBuilder;
    using Vector3Vec = StdVecAligned<Vector3>;


    boost::filesystem::path m_sceneFilePath;

    struct MassPoint {
        MassPoint(const RigidBodyState * s): m_state(s){}

        Vector3Vec * m_points = nullptr;
        unsigned int m_pointIdx = 0;
        unsigned int m_numPoints = 0;

        const RigidBodyState * m_state = nullptr; ///< Init Condition of the MassPoint pointing to m_initStates;
    };

    // LOCAL STUFF ==================================================================

    // For Binet Tensor
    Vector3 m_r_G_loc;       ///< Local Geometric center of all point masses
    Vector6 m_I_theta_loc;   ///< Local intertia tensor (binet tensor in center I) [ a_00,a_01,a_02,a_11,a_12,a_22] ) in intertial frame I
    AABB m_K_aabb_loc; ///< Local AABB in K Frame (oriented bounding box) , collaboratively solve the m_K_aabb_glo

    AABB m_I_aabb_loc;         ///< Local AABB of point masses
    std::vector<MassPoint> m_massPoints_loc; ///< local mass points
    Vector3Vec m_points_loc; /// point cloud of all predicted mass points
    unsigned int m_countPoints_loc = 0;



    // GLOBAL/LOCAL STUFF ===========================================================
    // references into this container remain valid, unordered_map, even if rehash
    RigidBodyStatesContainerType m_initStates; ///< All states received from all processes

    // Scene parser parses the initial time into this struct from global init condition
    struct TimeStepperSettings{ PREC m_startTime = 0;};
public:
    using TimeStepperSettingsType = TimeStepperSettings;
private:
    TimeStepperSettingsType m_timeStepperSettings;

    // GLOBAL STUFF =================================================================


    typename std::unordered_map<RankIdType, std::vector<const RigidBodyState *> > m_bodiesPerRank; ///< rank to all pointers in m_initStates;

    // For Binet Tensor
    Vector3 m_r_G_glo;         ///< Global geometric center ("G") of all point masses
    Vector6 m_I_theta_G_glo;   ///< Global intertia tensor (binet tensor in center point G) [a_00,a_01,a_02,a_11,a_12,a_22] ) in intertial frame I

    //Final Bounding Box
    bool m_aligned;         ///< aligned = AABB, otherwise OOBB
    AABB  m_aabb_glo;       ///< Min/Max of OOBB fit around points in frame K (OOBB) or I (AABB)
    Matrix33 m_A_IK;        ///< Transformation of OOBB cordinate frame K (oriented bounding box of point cloud) to intertia fram I

    std::unordered_map<RankIdType,AABB> m_rankAABBs;
    unsigned int m_countPoints_glo = 0;
    std::vector<MassPoint> m_massPoints_glo; ///< global mass points
    Vector3Vec m_points_glo; /// point cloud of all predicted mass points

    unsigned int m_nGlobalSimBodies = 0; ///< Only for a check with MassPoints

    using SettingsType = GridBuilderSettings;
    SettingsType m_settings;


    TopologyBuilderMessageWrapperResults<GridTopologyBuilder> m_messageWrapperResults;
    TopologyBuilderMessageWrapperOrientation<GridTopologyBuilder> m_messageOrient;
    TopologyBuilderMessageWrapperBodies<GridTopologyBuilder> m_messageBodies;
public:

    GridTopologyBuilder(std::shared_ptr<DynamicsSystemType> pDynSys,
                        std::shared_ptr<ProcessCommunicatorType > pProcCommunicator,
                        TopologyBuilderSettings::RebuildSettings rebuildSettings,
                        TopologyBuilderSettings::MassPointPredSettings massPointPredSettings,
                        SettingsType settings,
                        boost::filesystem::path sceneFilePath,
                        unsigned int nGlobalSimBodies)
        :TopologyBuilder(pDynSys, pProcCommunicator,rebuildSettings,massPointPredSettings), m_settings(settings),
        m_nGlobalSimBodies(nGlobalSimBodies),m_sceneFilePath(sceneFilePath),
        m_messageWrapperResults(this),m_messageOrient(this),m_messageBodies(this)
    {
        this->m_type = TopologyBuilderEnumType::GRIDBUILDER;

    }

    void cleanUpGlobal(){

        // Clean up;
        m_countPoints_glo = 0;
        m_massPoints_glo.clear();
        m_points_glo.clear();
        m_bodiesPerRank.clear();
        m_initStates.clear();
        m_rankAABBs.clear();

    }

    void cleanUpLocal(){

        // Clean up;
        m_countPoints_loc = 0;
        m_massPoints_loc.clear();
        m_initStates.clear();
        m_points_loc.clear();
    }


    void initTopology(PREC currentTime = 0) {
        LOGTB(m_pSimulationLog,"---> GridTopoBuilder: initialize Topology" <<std::endl;)

        // get initial startTime which was parsed in ( gets overwritten by global initial condition if specified to continue from)
        m_timeStepperSettings.m_startTime = m_pDynSys->getSettingsTimeStepper().m_startTime;

        // increment number of built topologies
        m_currentTime = currentTime;
        ++m_builtTopologies;

        RankIdType masterRank = this->m_pProcCommunicator->getMasterRank();
        if(this->m_pProcCommunicator->hasMasterRank()) {

            if(m_settings.m_processDim(0)*m_settings.m_processDim(1)*m_settings.m_processDim(2) != m_pProcCommunicator->getNProcesses()){
                ERRORMSG("processDim: " << m_settings.m_processDim << " does not fit "<<m_pProcCommunicator->getNProcesses()<<"processes!");
            }

            // Parse all initial condition from the scene file (global initial condition is read too)
            ParserModulesCreatorTopoBuilder<GridTopologyBuilder> c(this);
            SceneParserMPI<GridTopologyBuilder, ParserModulesCreatorTopoBuilder<GridTopologyBuilder>::template SceneParserTraits> parser(c, m_pSimulationLog) ; // this class is the modules generator

            // clean init states:
            m_initStates.clear();
            parser.parseScene(m_sceneFilePath);

            if(m_initStates.size() != m_nGlobalSimBodies){
                ERRORMSG("Parsed to little initial states in scene file: " << m_sceneFilePath << " states: " << m_initStates.size() << "globalSimBodies: " << m_nGlobalSimBodies <<std::endl);
            }

            LOGTBLEVEL2(m_pSimulationLog, "---> GridTopoBuilder: parsed states: "<<std::endl;);
            m_massPoints_glo.clear();
            for(auto & s : m_initStates){
                // Save mass points
                m_massPoints_glo.emplace_back( &s.second );
                LOGTBLEVEL3(m_pSimulationLog, "\t\t state @"<<&s.second <<" : "
                            << RigidBodyId::getBodyIdString(s.second.m_id )<< " , " << s.second.m_q.transpose() << std::endl;);

            }



            if(m_settings.m_buildMode == GridBuilderSettings::BuildMode::PREDEFINED){
                // build a static grid, set m_aabb_glo to settings from scene file.
                if(m_settings.m_aligned){
                    m_aabb_glo = m_settings.m_aabb;
                    m_A_IK.setIdentity();
                    m_aligned = true;
                }else{
                    m_aabb_glo = m_settings.m_aabb;
                    m_A_IK = m_settings.m_A_IK;
                    m_aligned = false;
                }
            }
            else{
                // build a fitted grid

                if(m_settings.m_buildMode == GridBuilderSettings::BuildMode::ALIGNED){
                    m_A_IK.setIdentity();
                    m_points_glo.clear();
                    predictMassPoints(m_massPoints_glo, m_points_glo );
                    m_countPoints_glo = buildCenterPoint(m_massPoints_glo,m_r_G_glo);
                    computeExtent<true>(m_massPoints_glo,m_aabb_glo);
                    m_aligned = true;

                }else if(m_settings.m_buildMode == GridBuilderSettings::BuildMode::BINET_TENSOR){

                    // Process for Grid Building with Binet Inertia Tensor  =================================================
                    m_points_glo.clear();
                    predictMassPoints(m_massPoints_glo, m_points_glo );
                    m_countPoints_glo = buildCenterPoint(m_massPoints_glo,m_r_G_glo);
                    buildBinetTensor(m_massPoints_glo,m_I_theta_G_glo, m_r_G_glo);
                    solveOrientation(m_A_IK,m_I_theta_G_glo);
                    computeExtent(m_massPoints_glo,m_aabb_glo,m_A_IK);
                    m_aligned = false;
                }
                else if(m_settings.m_buildMode == GridBuilderSettings::BuildMode::MVBB){
                    m_points_glo.clear();
                    predictMassPoints(m_massPoints_glo, m_points_glo );

                    makeMVBB(m_points_glo, m_A_IK, m_aabb_glo);

                    m_aligned = false;

                }
            }

            adjustGrid();

            // Write grid data to file
            #ifdef TOPOLOGY_BUILDER_WRITE_GRID
                #ifdef TOPOLOGY_BUILDER_WRITE_PREDICTED_POINTS
                    auto * p = &m_points_glo;
                #else
                    decltype(&m_points_glo) p = nullptr;
                #endif
                writeGridInfo(m_currentTime,m_aabb_glo,m_settings.m_processDim,m_aligned,m_A_IK,m_rankAABBs,p);
            #endif

            buildTopo();
            sendGrid(masterRank);
            cleanUpGlobal();
            cleanUpLocal();

        } else {

            // Receive all results from master;
            // All initial states get saved in m_pDynSys->m_bodiesInitStates
            m_messageWrapperResults.resetBeforLoad();
            m_pProcCommunicator->receiveMessageFromRank(m_messageWrapperResults,
                                                     this->m_pProcCommunicator->getMasterRank(),
                                                     m_messageWrapperResults.m_tag);

            buildTopo();
            cleanUpLocal();
        }

        LOGTBLEVEL1(m_pSimulationLog, "---> GridTopoBuilder: States in m_bodiesInitStates: (" << m_pDynSys->m_bodiesInitStates.size()<< ")" <<std::endl;);
            for(auto & s :  m_pDynSys->m_bodiesInitStates){
                LOGTBLEVEL3(m_pSimulationLog, "\t\t state: "
                            << RigidBodyId::getBodyIdString(s.second.m_id )<< " , q: "
                            << s.second.m_q.transpose() << " u: " << s.second.m_u.transpose() << std::endl;);
        }



        // Write m_startTime to TimeStepperSettings in DynamicsSystem (if it stayed the same , it does not matter)
        m_pDynSys->setStartTime( m_timeStepperSettings.m_startTime);


        LOGTB(m_pSimulationLog,"---> GridTopoBuilder: init topology finished!" <<std::endl;)
    }

    void rebuildTopology(PREC currentTime) {

        LOGTB(m_pSimulationLog,"---> GridTopologyBuilder: rebuild Topology" <<std::endl;)




        // increment number of built topologies
        ++m_builtTopologies;
        m_currentTime = currentTime;

        // Gather all body center of gravities and common AABB to master rank
        // Compute Inertia tensor where all masses of the points are equal to 1 (gives the tensor for the geometric center)
        buildLocalStuff();


        LOGTB(m_pSimulationLog,"---> GridTopoBuilder: Clear all init states in DynamicsSystem" <<std::endl;)
        m_pDynSys->m_bodiesInitStates.clear();

        RankIdType masterRank = this->m_pProcCommunicator->getMasterRank();
        if(this->m_pProcCommunicator->hasMasterRank()){

            // For sending
            m_bodiesPerRank.clear();
            m_rankAABBs.clear();


            // Receive Messages
            // Fill linear array with all ranks except master
            std::vector<RankIdType> ranks;
            unsigned int count=0;
            std::generate_n(std::back_inserter(ranks),
                            this->m_pProcCommunicator->getNProcesses()-1,
                            [&](){ if(count == masterRank){return (++count)++;}else{return count++;} });

            m_messageBodies.resetBeforLoad(); // Resets all variables for the appropriate build method in this class
            this->m_pProcCommunicator->receiveMessageFromRanks(m_messageBodies, ranks, m_messageBodies.m_tag);

            //Check if number of masspoints received is equal to the total in the simulation
            if(m_nGlobalSimBodies != m_massPoints_glo.size() + m_massPoints_loc.size()){
                ERRORMSG("m_nGlobalSimBodies: " << m_nGlobalSimBodies << "not equal to" <<  m_massPoints_glo.size() + m_massPoints_loc.size() <<std::endl);
            }



            if(m_settings.m_buildMode == GridBuilderSettings::BuildMode::ALIGNED){
                    m_A_IK.setIdentity();
                     //add own AABB to global
                    m_aabb_glo += m_I_aabb_loc;
                    m_aligned = true;

                    // DEBUG (predict points anyway, not needed! to check in file)
                    m_points_glo.clear();
                    predictMassPoints(m_massPoints_loc, m_points_glo );
                    predictMassPoints(m_massPoints_glo, m_points_glo );
            }
            else if(m_settings.m_buildMode == GridBuilderSettings::BuildMode::BINET_TENSOR){

                // Add our own center and weight it
                m_r_G_glo += m_r_G_loc * m_countPoints_loc;
                m_countPoints_glo += m_countPoints_loc;
                m_r_G_glo /= m_countPoints_glo;

                // Add our own tensor
                m_I_theta_G_glo += m_I_theta_loc;
                // Shift global to center!
                shiftBinetTensor(m_I_theta_G_glo,m_r_G_glo,m_countPoints_glo);

                solveOrientation(m_A_IK,m_I_theta_G_glo);
                collaborativeSolveExtent<true>(masterRank);
                m_aligned = false;

            }else if(m_settings.m_buildMode == GridBuilderSettings::BuildMode::MVBB){

                // Predict our local and global massPoints
                m_points_glo.clear();
                predictMassPoints(m_massPoints_loc, m_points_glo );
                predictMassPoints(m_massPoints_glo, m_points_glo );

                makeMVBB(m_points_glo, m_A_IK, m_aabb_glo);

                m_aligned = false;
            }

            adjustGrid();

            // Write grid data to file
            #ifdef TOPOLOGY_BUILDER_WRITE_GRID
                #ifdef TOPOLOGY_BUILDER_WRITE_PREDICTED_POINTS
                    auto * p = &m_points_glo;
                #else
                    decltype(&m_points_glo) p = nullptr;
                #endif
                writeGridInfo(m_currentTime,m_aabb_glo,m_settings.m_processDim,m_aligned,m_A_IK,m_rankAABBs,p);
            #endif

            buildTopo();
            sendGrid(masterRank);
            cleanUpGlobal();
            cleanUpLocal();

        }else{



            this->m_pProcCommunicator->sendMessageToRank(m_messageBodies, masterRank, m_messageBodies.m_tag);

            if(m_settings.m_buildMode == GridBuilderSettings::BuildMode::BINET_TENSOR){
                collaborativeSolveExtent<false>(masterRank);
            }

            // Receive Grid results from master;
            // All initial states get saved in m_pDynSys->m_bodiesInitStates
            m_messageWrapperResults.resetBeforLoad();
            m_pProcCommunicator->receiveMessageFromRank( m_messageWrapperResults,
                                                         masterRank,
                                                         m_messageWrapperResults.m_tag);

            buildTopo();
            cleanUpLocal();
        }

        LOGTBLEVEL1(m_pSimulationLog, "---> GridTopoBuilder: States in m_bodiesInitStates: (" << m_pDynSys->m_bodiesInitStates.size()<< ")" <<std::endl;);
            for(auto & s :  m_pDynSys->m_bodiesInitStates){
                LOGTBLEVEL3(m_pSimulationLog, "\t\t state: "
                            << RigidBodyId::getBodyIdString(s.second.m_id )<< " , q: "
                            << s.second.m_q.transpose() << " u: " << s.second.m_u.transpose() << std::endl;);
        }

        LOGTB(m_pSimulationLog,"---> GridTopoBuilder: rebuild topology finished!" <<std::endl;)
    }

    void adjustGrid(){

        Array3 e = m_aabb_glo.extent();

        // Sort process dim according to extent (if needed)
        if(m_settings.m_matchProcessDimToExtent){
            // Sort processDim ascending
            SettingsType::ProcessDimType procDim = m_settings.m_processDim;
            std::sort(procDim.data(),procDim.data() + procDim.size());

            // Get min extent idx
            Array3::Index minIdx;
            e.minCoeff(&minIdx);

            // set min proc dim for min extent
            m_settings.m_processDim(minIdx) = procDim(0);
            // set proc dim for remaining
            unsigned int i1 = (minIdx+1)%3;
            unsigned int i2 = (minIdx+2)%3;
            if( e(i1) <= e(i2) ){
                m_settings.m_processDim(i1) = procDim(1);
                m_settings.m_processDim(i2) = procDim(2);
            }else{
                m_settings.m_processDim(i1) = procDim(2);
                m_settings.m_processDim(i2) = procDim(1);
            }
        }

        //Adjust box to minimal box size =  procDim* min_gridSize,
        Array3 limits = m_settings.m_processDim.cast<PREC>() * m_settings.m_minGridSize;
        m_aabb_glo.expandToMinExtentAbsolute( limits );
    }

    void buildLocalStuff() {

        // we copy all body points into the set m_initStates
        m_initStates.clear();
        m_massPoints_loc.clear();

        for(auto & body : m_pDynSys->m_simBodies){
            auto res = m_initStates.emplace(body->m_id,body); // add new state
            //applyBody(body); //add state
            m_massPoints_loc.emplace_back( &(res.first->second) ); // pointer stays valid also if rehash of m_initStates
            LOGTBLEVEL3(m_pSimulationLog, "\t\t state: " << RigidBodyId::getBodyIdString(body->m_id )<< " , " <<  res.first->second.m_q.transpose() << std::endl;);
        }

        if(m_settings.m_buildMode == GridBuilderSettings::BuildMode::BINET_TENSOR){

            // Predict masspoints over some timesteps (simple)
            m_points_loc.clear();
            predictMassPoints(m_massPoints_loc, m_points_loc);

            m_countPoints_loc = buildCenterPoint(m_massPoints_loc,m_r_G_loc);
            buildBinetTensor<false>(m_massPoints_loc,m_I_theta_loc);
            // Move inertia tensor to geometric center G (steiner formula) on root process!
            computeExtent<true>(m_massPoints_loc,m_I_aabb_loc);
        }else if(m_settings.m_buildMode == GridBuilderSettings::BuildMode::ALIGNED){

            // Predict masspoints over some timesteps (simple)
            m_points_loc.clear();
            predictMassPoints(m_massPoints_loc, m_points_loc);
            computeExtent<true>(m_massPoints_loc,m_I_aabb_loc);

        }else if(m_settings.m_buildMode == GridBuilderSettings::BuildMode::MVBB){

        }else{
            ERRORMSG("This should not happen! Undefined BuildMode in Rebuilding")
        }


    }

    // Compute Minimum Bounding Box with ApproxMVBB source code
    void makeMVBB(Vector3Vec & points, Matrix33 & A_IK, AABB & aabb ){

        LOGTBLEVEL3(m_pSimulationLog, "Points MVBB \n:");
        m_aabb_glo.reset();
        for(auto & p: points){
             m_aabb_glo.unite(p);
             LOGTBLEVEL3(m_pSimulationLog, p.transpose().format(MyMatrixIOFormat::SpaceSep) << "\n");
        }
         LOGTBLEVEL3(m_pSimulationLog, std::endl;);

        // Map points in std::vector to matrix
        if(sizeof(Vector3) != 3*sizeof(PREC)){
            ERRORMSG("There is padding inside Vector3 which is not allowed here")
        }

        MatrixMap<const ApproxMVBB::Matrix3Dyn> p(points.data()->data(), 3, points.size());

        PREC epsilon = 0.01*m_aabb_glo.extent().matrix().norm(); // percentage of the approximate extent of the point cloud
        OOBB oobb = ApproxMVBB::approximateMVBB(p,epsilon,400,5,0,6);

        // Set A_IK
        A_IK = oobb.m_q_KI.matrix();

        // set min/max in K frame
        aabb.m_minPoint = oobb.m_minPoint;
        aabb.m_maxPoint = oobb.m_maxPoint;

        if(aabb.isEmpty()){
            ERRORMSG("Box is empty")
        }
        LOGTBLEVEL3( m_pSimulationLog, "---> GridTopoBuilder: Coordinate Frame: "<<std::endl<< A_IK <<std::endl; );
        LOGTBLEVEL3( m_pSimulationLog, "---> GridTopoBuilder: A_IK * A_IK^T: "<<std::endl<< A_IK*A_IK.transpose() <<std::endl; );
    }

    void solveOrientation(Matrix33 & A_IK, const Vector6 & theta_G){
        //Calculate principal axis of Theta_G
        Matrix33 Theta_G;
        MatrixHelpers::setSymMatrix(Theta_G,theta_G);


        LOGTBLEVEL3( m_pSimulationLog, "---> GridTopoBuilder: Global Binet inertia tensor: " << std::endl
                      << "\t\t " << m_I_theta_G_glo(0) << "\t" << m_I_theta_G_glo(1) << "\t" << m_I_theta_G_glo(2) << std::endl
                      << "\t\t " << m_I_theta_G_glo(1) << "\t" << m_I_theta_G_glo(3) << "\t" << m_I_theta_G_glo(4) << std::endl
                      << "\t\t " << m_I_theta_G_glo(2) << "\t" << m_I_theta_G_glo(4) << "\t" << m_I_theta_G_glo(5) << std::endl;)

        typename MyMatrixDecomposition::template EigenSolverSelfAdjoint<Matrix33> eigenSolv(Theta_G);

        A_IK = eigenSolv.eigenvectors();
        LOGTBLEVEL3( m_pSimulationLog, "---> GridTopoBuilder: Eigenvalues: "<<std::endl<< eigenSolv.eigenvalues().transpose() <<std::endl; );
        LOGTBLEVEL3( m_pSimulationLog, "---> GridTopoBuilder: Eigenvectors: "<<std::endl<< A_IK <<std::endl; );

        //        // Correct A_IK to be as orthogonal as possible, project x onto x-y plane, (for no use so far)
        //        A_IK.col(2).normalize();
        //        A_IK.col(0) -= A_IK.col(2).dot(A_IK.col(0)) * A_IK.col(2); A_IK.col(0).normalize();
        //        A_IK.col(1) = A_IK.col(2).cross(A_IK.col(0));              A_IK.col(1).normalize();

        // Coordinate frame of OOBB
        if(A_IK(2,2) <= 0){ // if z-Axis is negativ -> *-1
            A_IK.col(1).swap(A_IK.col(0));
            A_IK.col(2) *= -1.0;
        }
        LOGTBLEVEL3( m_pSimulationLog, "--->GridTopologyBuilder: Coordinate Frame: "<<std::endl<< A_IK <<std::endl; );
        LOGTBLEVEL3( m_pSimulationLog, "--->GridTopologyBuilder: x*y,y*z,z*x: "<<
                    A_IK.col(0).dot(A_IK.col(1)) <<", "<<
                    A_IK.col(1).dot(A_IK.col(2)) <<", "<<
                    A_IK.col(2).dot(A_IK.col(0))  <<std::endl; );
    }

    template<bool isRoot>
    void collaborativeSolveExtent(RankIdType masterRank){

        LOGTBLEVEL3(m_pSimulationLog, "---> GridTopoBuilder: Solve extent collaborative ");
        // Get orientation
        if(isRoot){
            // broadcast orientation
            m_pProcCommunicator->sendBroadcast(m_messageOrient);
        }else{
            m_pProcCommunicator->receiveBroadcast(m_messageOrient,masterRank);
        }

       LOGTBLEVEL3(m_pSimulationLog, "---> GridTopoBuilder: Compute extent locally ");
       computeExtent(m_massPoints_loc, m_K_aabb_loc, m_A_IK);

       LOGTBLEVEL3(m_pSimulationLog, "---> GridTopoBuilder: Do reduction with local extents");
        // Do a reduction of min and max values of the AABBs in K frame
        if(isRoot){ // ROOT
            m_pProcCommunicator->reduce(m_K_aabb_loc.m_minPoint, m_aabb_glo.m_minPoint, MPILayer::ReduceFunctions::MinVector3);
            m_pProcCommunicator->reduce(m_K_aabb_loc.m_maxPoint, m_aabb_glo.m_maxPoint, MPILayer::ReduceFunctions::MaxVector3);
        }else{ //All others
            m_pProcCommunicator->reduce(m_K_aabb_loc.m_minPoint, MPILayer::ReduceFunctions::MinVector3, masterRank);
            m_pProcCommunicator->reduce(m_K_aabb_loc.m_maxPoint, MPILayer::ReduceFunctions::MaxVector3, masterRank);
        }

    }

    template<bool aligned = false>
    void computeExtent(const std::vector<MassPoint> & massPoints, AABB & aabb, const Matrix33 & A_IK = Matrix33::Identity()){
        LOGTBLEVEL3(m_pSimulationLog, "---> GridTopoBuilder: Compute extent ..." << std::endl;);
        // Calculate min max
        aabb.reset();
        Vector3 t;
        for(auto & massPoint : massPoints){
            // loop over all prediction points of this mass point
            for(unsigned int i = massPoint.m_pointIdx ; i < massPoint.m_pointIdx + massPoint.m_numPoints; ++i){

                if(aligned){ // ignore transformation!
                    aabb += (*massPoint.m_points)[i];
                }else{
                    t = A_IK.transpose() * (*massPoint.m_points)[i];
                    aabb += t;
                }
            }
        }
    }


    // Predict Bodies over some steps
    // Either use Fluidix to quickly approximate a simulation or
    // do a simple integration with only gravity of all point masses, stop if collided with non simulated bodies
    void predictMassPoints(std::vector<MassPoint> & massPoints,
                           Vector3Vec & predictedPoints){


        ColliderPoint pointCollider;

        PREC dT = m_massPointPredSettings.m_deltaT;

        unsigned int nPoints = m_massPointPredSettings.m_nPoints;

        auto * gravityField = m_pDynSys->m_externalForces.getGravityField();
        Vector3 gravity = Vector3::Zero();
        if( gravityField ){
            gravity= gravityField->getGravity();
        }

        auto & staticBodies = m_pDynSys->m_staticBodies;

        for(auto & massPoint : massPoints){

            ASSERTMSG(massPoint.m_state,"State null");

            massPoint.m_points = &predictedPoints;
            massPoint.m_pointIdx = predictedPoints.size();  // link index of the predicted points
            massPoint.m_numPoints = 1; // set number of points

            predictedPoints.push_back(massPoint.m_state->getPosition());

            Vector3 vel = massPoint.m_state->getVelocityTrans();
            // Predict mass points with gravity, no collision detection so far with static objects
            // r_s(t) = 1/2*g*t^2 + v * t + r_s
            for(unsigned int i = 0; i <nPoints; i++ ){

                predictedPoints.push_back( predictedPoints.back() + dT * vel + 0.5*gravity*dT*dT );
                vel +=  gravity*dT;

                massPoint.m_numPoints += 1;

                // intersect with all static bodies, if intersection abort, point is proxed onto body!
                bool hitObject = false;
                for(auto & pBody : staticBodies){

                    if( pointCollider.intersectAndProx(pBody, predictedPoints.back() ) ){
                        hitObject = true;
                        // dont break, prox to all static objects, (corners are tricky!)
                    }
                }

                if(hitObject){
                    break;
                }

            }

            LOGTBLEVEL3(m_pSimulationLog, "---> GridTopoBuilder: body: "
            <<RigidBodyId::getBodyIdString(massPoint.m_state->m_id) << " last prediction: " << predictedPoints.back().transpose()  <<std::endl);
        }

        LOGTBLEVEL2(m_pSimulationLog, "---> GridTopoBuilder: Predicted mass points: " << predictedPoints.size() << std::endl;);
    }

    unsigned int buildCenterPoint( const std::vector<MassPoint> & massPoints,
                                   Vector3 & center){
         LOGTBLEVEL3(m_pSimulationLog, "---> GridTopoBuilder: Build center point ..."<< std::endl;);
        unsigned int countPoints = 0;
        center.setZero();
        for(auto & massPoint : massPoints){

            // loop over all prediction points of this mass point
            for(unsigned int i = massPoint.m_pointIdx ; i < massPoint.m_pointIdx + massPoint.m_numPoints; ++i){
                // Center of masses
                center += (*massPoint.m_points)[i];
                ++countPoints;
            }
        }
        center /= countPoints;

        return countPoints;
    }


    template<bool shift = true>
    unsigned int buildBinetTensor( const std::vector<MassPoint> & massPoints,
                                   Vector6 & I_theta_G,
                                   const Vector3 & I_r_IG = Vector3::Zero()){
         LOGTBLEVEL3(m_pSimulationLog, "---> GridTopoBuilder: Build Binet Tensor ..."<< std::endl;);
        unsigned int countPoints = 0;
        I_theta_G.setZero();
        for(auto & massPoint : massPoints){
            // loop over all prediction points of this mass point
           for(unsigned int i = massPoint.m_pointIdx ; i < massPoint.m_pointIdx + massPoint.m_numPoints; ++i){
                Vector3 & r_IS = (*massPoint.m_points)[i];
                // Binet Tensor (expressed in the frame I of the massPoints)  (mass=1, with reference to 0 in Interial System I) calculate only the 6 essential values
                I_theta_G(0) += r_IS(0)*r_IS(0);
                I_theta_G(1) += r_IS(0)*r_IS(1);
                I_theta_G(2) += r_IS(0)*r_IS(2);
                I_theta_G(3) += r_IS(1)*r_IS(1);
                I_theta_G(4) += r_IS(1)*r_IS(2);
                I_theta_G(5) += r_IS(2)*r_IS(2);

                ++countPoints;
            }
        }
        if(shift){
            shiftBinetTensor(I_theta_G, I_r_IG, countPoints );
        }
    }

    void shiftBinetTensor(Vector6 & I_theta_G,
                          const Vector3 & I_r_IG, PREC scalar)
    {
            // Move intertia tensor to center G
            I_theta_G(0) -= I_r_IG(0)*I_r_IG(0)  * scalar;
            I_theta_G(1) -= I_r_IG(0)*I_r_IG(1)  * scalar;
            I_theta_G(2) -= I_r_IG(0)*I_r_IG(2)  * scalar;
            I_theta_G(3) -= I_r_IG(1)*I_r_IG(1)  * scalar;
            I_theta_G(4) -= I_r_IG(1)*I_r_IG(2)  * scalar;
            I_theta_G(5) -= I_r_IG(2)*I_r_IG(2)  * scalar;
    }

    void buildTopo(){

        LOGTBLEVEL1(m_pSimulationLog, "---> GridTopoBuilder: GridTopologyBuilder, build grid ..." << std::endl;);

            if(m_rebuildSettings.m_mode == RebuildSettings::Mode::STATIC){
                LOGTBLEVEL1( m_pSimulationLog, "---> GridTopoBuilder: Create ProcessTopologyGrid Static: " << std::endl;)
            }
            else{
                LOGTBLEVEL1( m_pSimulationLog, "---> GridTopoBuilder: Create ProcessTopologyGrid Dynamic: "<< std::endl;)
            }

            if(  m_settings.m_buildMode == GridBuilderSettings::BuildMode::ALIGNED){
                 LOGTBLEVEL1( m_pSimulationLog,"\t buildMode: ALIGNED" << std::endl);
            }else if(m_settings.m_buildMode == GridBuilderSettings::BuildMode::PREDEFINED){
                 LOGTBLEVEL1( m_pSimulationLog, "\t buildMode: PREDEFINED" << std::endl );
            }else if (m_settings.m_buildMode == GridBuilderSettings::BuildMode::BINET_TENSOR){
                 LOGTBLEVEL1( m_pSimulationLog,"\t buildMode: BINET" << std::endl);
            }else if(m_settings.m_buildMode == GridBuilderSettings::BuildMode::MVBB){
                 LOGTBLEVEL1( m_pSimulationLog,"\t buildMode: MVBB" << std::endl);
            }else{
                ERRORMSG("No BUILD implemented!")
            }



            LOGTBLEVEL1( m_pSimulationLog,
                            "\t aligned: "<< m_settings.m_aligned << std::endl <<
                            "\t min/max: "<<
                            " [ " << m_aabb_glo.m_minPoint.transpose() << " , " << m_aabb_glo.m_maxPoint.transpose() << "]" << std::endl <<
                            "\t A_IK: \n" << m_A_IK << std::endl <<
                            "\t dim: " << "[ " << m_settings.m_processDim.transpose() << "]" << std::endl <<
                            "\t extent: " << "[ " << m_aabb_glo.extent().transpose() << "]" << std::endl;);
            m_pProcCommunicator->createProcTopoGrid(m_aabb_glo,
                                                    m_settings.m_processDim,
                                                    m_aligned,
                                                    m_A_IK);

            LOGTBLEVEL1( m_pSimulationLog,"---> GridTopoBuilder: Initialize ProcessCommunicator buffers..." << std::endl);
            m_pProcCommunicator->initializeNeighbourBuffers();

    }

    //only master rank executes this
    void sendGrid(RankIdType masterRank) {


        sortBodies(masterRank);

        // Send all data to the neighbours ===========================
        // For each rank r send:
        // - topology identifier (GRID)
        // - topology settings (GRID: min,max,dimension)
        // - send all body ids and initial states for the bodies in rank r


        // Fill linear array with all ranks except master
        std::vector<RankIdType> ranks;
        unsigned int count=0;
        std::generate_n(std::back_inserter(ranks), this->m_pProcCommunicator->getNProcesses()-1, [&](){ if(count ==masterRank){return (++count)++;}else{return count++;} });


        for(auto rank : ranks){
            LOGTBLEVEL1( m_pSimulationLog, "---> GridTopoBuilder: Sending grid to rank: "<< rank <<std::endl; );
            m_messageWrapperResults.setRank(rank);
            m_pProcCommunicator->sendMessageToRank(m_messageWrapperResults,rank, m_messageWrapperResults.m_tag);
        }
        // ===========================================================


        // Safty check:
        ASSERTMSG(m_bodiesPerRank.size() == 0,"All states should be sent!");
    }

    // master sorts bodies
    void sortBodies(RankIdType masterRank ){
        LOGTBLEVEL3( m_pSimulationLog, "---> GridTopoBuilder: Sort bodies " << std::endl;);

        // Sort states acoording to their rank their in!
        m_bodiesPerRank.clear();
        m_pDynSys->m_bodiesInitStates.clear();

        for(auto & p : m_initStates){
            RankIdType ownerRank;
            m_pProcCommunicator->getProcTopo()->belongsPointToProcess(p.second.getPosition(),ownerRank);

            if(ownerRank == masterRank){
                // move own body states in dynamic system init states
                m_pDynSys->m_bodiesInitStates.insert(std::make_pair(p.second.m_id,p.second));
            }else{
                // move other body states to a list for sending
                m_bodiesPerRank[ownerRank].push_back(&p.second);
            }

        }

        // Log stuff
        std::stringstream ss;
        for(auto & s : m_bodiesPerRank){
           LOGTBLEVEL3(m_pSimulationLog, "\t Bodies for rank " << s.first << " : ");
            for(auto & state : s.second){
                LOGTBLEVEL3(m_pSimulationLog,  RigidBodyId::getBodyIdString(state->m_id) << " , ");
            }
            LOGTBLEVEL3(m_pSimulationLog, std::endl);
        }
    }


    void writeGridInfo(PREC currentTime,
                       const AABB & aabb,
                       const typename GridBuilderSettings::ProcessDimType & dim,
                       bool aligned,
                       const Matrix33 & A_IK,
                       const std::unordered_map<RankIdType,AABB> & rankToAABB,
                       Vector3Vec * points = nullptr
                       )
    {
        boost::filesystem::path filePath = m_localSimFolderPath;

        std::string filename = "TopologyInfo_" + std::to_string(m_builtTopologies);
        filePath /= filename + ".xml";

        LOGTBLEVEL3(m_pSimulationLog, "Write Grid Data to: " << filePath <<std::endl;);

        // Open XML and write structure!
        pugi::xml_document dataXML;
        std::stringstream xml("<TopologyBuilder type=\"Grid\" buildMode=\"\" >"
                                    "<Description>"
                                        "A_IK is tranformation matrix, which transforms points from Frame K to Frame I\n"
                                        "AABB is in K Frame\n"
                                        "AABBList contains all AABBs from all ranks\n"
                                    "</Description>"
                                    "<Time value=\"\" />"
                                    "<AABBList />"
                                    "<Grid aligned=\"\" >"
                                        "<Dimension/>"
                                        "<MinPoint/>"
                                        "<MaxPoint/>"
                                        "<A_IK/>"
                                    "</Grid>"
                                    "<Points/>"
                                "</TopologyBuilder>");

        bool r = dataXML.load(xml);
        ASSERTMSG(r,"Could not load initial xml data file");

        // Write data

        using XMLNodeType = pugi::xml_node;
        XMLNodeType node;
        static const auto  XMLStringNodeType = pugi::node_pcdata;
        XMLNodeType root =  dataXML.child("TopologyBuilder");


        std::string buildMode ="nothing";

        if(  m_settings.m_buildMode == GridBuilderSettings::BuildMode::ALIGNED){
                 buildMode="Aligned";
        }else if(m_settings.m_buildMode == GridBuilderSettings::BuildMode::PREDEFINED){
             buildMode="Predefined";
        }else if (m_settings.m_buildMode == GridBuilderSettings::BuildMode::BINET_TENSOR){
             buildMode="BinetTensor";
        }else if(m_settings.m_buildMode == GridBuilderSettings::BuildMode::MVBB){
             buildMode="MVBB";
        }

        root.attribute("buildMode").set_value(buildMode.c_str()) ;

        root.first_element_by_path("./Grid").attribute("aligned").set_value( aligned );

        node = root.first_element_by_path("./Time");
        node.attribute("value").set_value( std::to_string(currentTime).c_str() );

        node = root.first_element_by_path("./Grid/MinPoint");
        node.append_child(XMLStringNodeType).set_value( Utilities::typeToString(aabb.m_minPoint.transpose().format(MyMatrixIOFormat::SpaceSep)).c_str() );

        node = root.first_element_by_path("./Grid/MaxPoint");
        node.append_child(XMLStringNodeType).set_value( Utilities::typeToString(aabb.m_maxPoint.transpose().format(MyMatrixIOFormat::SpaceSep)).c_str() );

        node = root.first_element_by_path("./Grid/A_IK");
        node.append_child(XMLStringNodeType).set_value( Utilities::typeToString(A_IK.format(MyMatrixIOFormat::SpaceSep)).c_str() );

        node = root.first_element_by_path("./Grid/Dimension");
        node.append_child(XMLStringNodeType).set_value( Utilities::typeToString(dim.transpose().format(MyMatrixIOFormat::SpaceSep)).c_str() );

        node = root.first_element_by_path("./Grid/AABBList");
        std::stringstream ss;
        for(auto & aabbIt : rankToAABB){

            ss << aabbIt.first
            << " " << aabbIt.second.m_minPoint.transpose().format(MyMatrixIOFormat::SpaceSep)
            << " " << aabbIt.second.m_maxPoint.transpose().format(MyMatrixIOFormat::SpaceSep)
            << std::endl;

        }
        node.append_child(XMLStringNodeType).set_value( ss.str().c_str() );

        // Write points
        if(points){
            ss.str("");
            node = root.first_element_by_path("./Points");
            for(auto & p : *points){
                ss << p.transpose().format(MyMatrixIOFormat::SpaceSep) << std::endl;
            }
            node.append_child(XMLStringNodeType).set_value( ss.str().c_str() );
        }

        dataXML.save_file(filePath.c_str(),"    ");
    }

};

}; //MPILayer


#endif

