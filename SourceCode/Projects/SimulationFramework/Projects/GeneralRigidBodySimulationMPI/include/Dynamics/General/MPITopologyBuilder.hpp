#ifndef MPITopologyBuilder_hpp
#define MPITopologyBuilder_hpp

#include <mpi.h>

#include "AssertionDebug.hpp"
#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include "MPIMessages.hpp"

#include "MPICommunication.hpp"

#include DynamicsSystem_INCLUDE_FILE

#include "MPITopologyBuilderSettings.hpp"

namespace MPILayer {


class TopologyBuilder {

public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
    DEFINE_MPI_INFORMATION_CONFIG_TYPES

    using TopologyBuilderEnumType = TopologyBuilderEnum;
    typedef typename MPILayer::ProcessCommunicator ProcessCommunicatorType;

    TopologyBuilder(std::shared_ptr<DynamicsSystemType> pDynSys,
                    std::shared_ptr<ProcessCommunicatorType > pProcCommunicator):
        m_pDynSys(pDynSys), m_pProcCommunicator(pProcCommunicator) {

    }

    virtual void initTopology() {};
    virtual void rebuildTopology() {};

    virtual ~TopologyBuilder() {}

protected:


    std::shared_ptr<DynamicsSystemType>  m_pDynSys;
    std::shared_ptr<ProcessCommunicatorType> m_pProcCommunicator;


    TopologyBuilderEnumType m_type;
};


template<typename TSceneParser, typename TProcCommunicator>
class GridTopologyBuilder : public TopologyBuilder {

    template<typename TTopologyBuilder >
    friend class TopologyBuilderMessageWrapperBodies;

public:
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
    DEFINE_MPI_INFORMATION_CONFIG_TYPES

    typedef TProcCommunicator ProcessCommunicatorType;
    using SceneParserType = TSceneParser;

    GridTopologyBuilder(std::shared_ptr<DynamicsSystemType> pDynSys,
                        std::shared_ptr<ProcessCommunicatorType > pProcCommunicator,
                        GridBuilderSettings settings,
                        boost::filesystem::path sceneFilePath,
                        unsigned int nGlobalSimBodies)
        :TopologyBuilder(pDynSys, pProcCommunicator), m_settings(settings),
        m_nGlobalSimBodies(nGlobalSimBodies),m_sceneFilePath(sceneFilePath)
    {
        this->m_type = TopologyBuilderEnumType::GRIDBUILDER;

        m_pSimulationLog = nullptr;
        m_pSimulationLog = Logging::LogManager::getSingletonPtr()->getLog("SimulationLog");
        ASSERTMSG(m_pSimulationLog, "There is no SimulationLog in the LogManager!");

    }

    void initTopology() {
        LOGTB(m_pSimulationLog,"---> initialize Topology (Grid)" <<std::endl;)

        if(this->m_pProcCommunicator->hasMasterRank()) {

            // Parse all initial condition from the scene file ================
            SceneParserType parser(*this) ; // this class is the modules generator
            typename SceneParserType::BodyModuleOptionsType o;
            o.m_parseAllBodiesNonSelGroup = true;
            o.m_parseSimBodies = true; o.m_allocateSimBodies = false;
            o.m_parseStaticBodies = false; o.m_allocateStaticBodies = false;
            // clean init states:
            m_initStates.clear();
            parser.parseScene(m_sceneFilePath, typename SceneParserType::SceneParserOptionsType(), o);

            if(m_initStates.size() != m_nGlobalSimBodies){
                ERRORMSG("Parsed to little initial states in scene file: " << m_sceneFilePath << std::endl);
            }

            LOGTBLEVEL3(m_pSimulationLog, "---> parsed states: "<<std::endl;);
            for(auto & s : m_initStates){
                // Save mass points
                m_massPoints_glo.insert(std::make_pair(s.first, MassPoint(s.second,m_nPointsPredictor) ));
                LOGTBLEVEL3(m_pSimulationLog, "\t\t state: "
                            << RigidBodyId::getBodyIdString(s.second.m_id )<< " , " << s.second.m_q.transpose() << std::endl;);
            }
            // ================================================================

            predictMassPoints();

            // Preprocess for Grid Building =================================================
            m_r_G_loc.setZero();
            m_aabb_glo.reset(); // Reset total AABB
            m_I_theta_G_glo.setZero();

            unsigned int countPoints = 0;
            for(auto & massPoint : m_massPoints_glo){

                // loop over all prediction points of this mass point
                for(auto & r_IS : massPoint.second.m_points){
                    // Global AABB
                    m_aabb_glo.unite(r_IS);
                    // Center of masses
                    m_r_G_glo += r_IS;

                    // Binet Tensor   (mass=1, with reference to Interial System I) calculate only the 6 essential values
                    m_I_theta_G_glo(0) += r_IS(0)*r_IS(0);
                    m_I_theta_G_glo(1) += r_IS(0)*r_IS(1);
                    m_I_theta_G_glo(2) += r_IS(0)*r_IS(2);
                    m_I_theta_G_glo(3) += r_IS(1)*r_IS(1);
                    m_I_theta_G_glo(4) += r_IS(1)*r_IS(2);
                    m_I_theta_G_glo(5) += r_IS(2)*r_IS(2);

                    ++countPoints;
                }
            }
            m_r_G_glo /= countPoints;
            // Move intertia tensor to center G
//            m_I_theta_G_glo(0) -= m_r_G_glo(0)*m_r_G_glo(0);
//            m_I_theta_G_glo(1) -= m_r_G_glo(0)*m_r_G_glo(1);
//            m_I_theta_G_glo(2) -= m_r_G_glo(0)*m_r_G_glo(2);
//            m_I_theta_G_glo(3) -= m_r_G_glo(1)*m_r_G_glo(1);
//            m_I_theta_G_glo(4) -= m_r_G_glo(1)*m_r_G_glo(2);
//            m_I_theta_G_glo(5) -= m_r_G_glo(2)*m_r_G_glo(2);


            // ================================================================

            buildGrid();


        } else {
            // Do not send data the first time
            // Wait for the grid and initial conditions
        }

        m_pProcCommunicator->waitBarrier();

    }

    void rebuildTopology() {
        /*
        // Gather all body center of gravities and common AABB to master rank
        // Compute Inertia tensor where all masses of the points are equal to 1 (gives the tensor for the geometric center)
        buildLocalStuff();




        TopologyBuilderMessageWrapperBodies<GridTopologyBuilder> m(this);
        RankIdType masterRank = this->m_pProcCommunicator->getMasterRank();
        if(this->m_pProcCommunicator->hasMasterRank()){

            // Rest global stuff
            m_aabb_glo.reset(); // Reset total AABB
            m_theta_G_glo.setZero();
            m_massPoints_glo.clear();
            m_initStates.clear();
            m_bodiesPerRank.clear();

            // Fill linear array with all ranks except master
            std::vector<RankIdType> ranks;
            unsigned int count=0;
            std::generate_n(std::back_inserter(ranks),
                            this->m_pProcCommunicator->getNProcesses(),
                            [&](){ if(count == masterRank){return (++count)++;}else{return count++;} });

            this->m_pProcCommunicator->receiveMessageFromRanks(m, ranks, m.m_tag);

            //Check if number of masspoints received is equal to the total in the simulation
            if(m_nGlobalSimBodies != m_massPoints_glo.size()){
                ERRORMSG("m_nGlobalSimBodies: " << m_nGlobalSimBodies << "not equal to" << m_massPoints_glo.size() <<std::endl);
            }

            // Weight the geometric center
            m_r_G_glo /= m_massPoints_glo.size();

            buildGrid();

            // BroadCast information (Grid and InitalStates)



        }else{
            this->m_pProcCommunicator->sendMessageToRank(m, masterRank, m.m_tag);
        }
        */
    }


    void buildLocalStuff() {

        m_aabb_loc.reset();
        m_r_G_loc.setZero();
        m_I_theta_loc.setZero();

        // we copy all body points into the set m_points_loc
        // and add the number points according to the timesteps the predictor integrator is generating



        // calculate geometric center and AABB and also inertia tensor (first with respect to interia frame,
        // then we move it with steiner formula to geometric center)
        for(auto & body : m_pDynSys->m_SimBodies) {

            m_aabb_loc.unite((body)->m_r_S);

            m_r_G_loc += (body)->m_r_S;

            // Binet Tensor (with reference to Interial System I) calculate only the 6 essential values
#define r_IS  (body)->m_r_S
            m_I_theta_loc(0) += r_IS(0)*r_IS(0);
            m_I_theta_loc(1) += r_IS(0)*r_IS(1);
            m_I_theta_loc(2) += r_IS(0)*r_IS(2);
            m_I_theta_loc(3) += r_IS(1)*r_IS(1);
            m_I_theta_loc(4) += r_IS(1)*r_IS(2);
            m_I_theta_loc(5) += r_IS(2)*r_IS(2);
#undef r_IS
        }

        m_r_G_loc /= this->m_pDynSys->m_SimBodies.size();
        // Move inertia tensor to geometric center (steiner formula) on root!

        // Predict Bodies over some steps
        // Either use Fluidix to quickly approximate a simulation or do a simple integration with only gravity of all point masses, stop if collided with non simulated bodies
    }


    void predictMassPoints(){
        for(auto & massPoint : m_massPoints_glo){

            Vector3 vel = massPoint.second.m_state.getVelocityTrans();
            auto & points = massPoint.second.m_points;

            // Predict mass points linearly, no collision detection so far with static objects
            ASSERTMSG(points.size()== m_nPointsPredictor+1, "number of points wrong!")
            for(unsigned int i = 1; i< m_nPointsPredictor; i++ ){
                points[i] = points[i-1] + m_deltaT*vel;
            }

        }
    }

    //only master rank
    void buildGrid() {

        LOGTBLEVEL1(m_pSimulationLog, "---> Global center of mass points: " << m_r_G_glo.transpose() << std::endl
                                      << "---> Global AABB of mass points: min: " << m_aabb_glo.m_minPoint.transpose()
                                                                       << "max: " << m_aabb_glo.m_maxPoint.transpose() << std::endl
                                      << "---> Global Binet inertia tensor: " << std::endl
                                      << "\t\t " << m_I_theta_G_glo(0) << "\t" << m_I_theta_G_glo(1) << "\t" << m_I_theta_G_glo(2) << std::endl
                                      << "\t\t " << m_I_theta_G_glo(1) << "\t" << m_I_theta_G_glo(3) << "\t" << m_I_theta_G_glo(4) << std::endl
                                      << "\t\t " << m_I_theta_G_glo(2) << "\t" << m_I_theta_G_glo(4) << "\t" << m_I_theta_G_glo(5) << std::endl;
                    )

        //Calculate principal axis of Theta_G
        Matrix33 Theta_G;
        MatrixHelpers::setSymMatrix(Theta_G,m_I_theta_G_glo);
        typename MyMatrixDecomposition::template EigenSolverSelfAdjoint<Matrix33> eigenSolv(Theta_G);

        Matrix33 A_IK = eigenSolv.eigenvectors();
        LOGTBLEVEL3( m_pSimulationLog, "Eigenvalues: "<<std::endl<< eigenSolv.eigenvalues() <<std::endl; );
        LOGTBLEVEL3( m_pSimulationLog, "Eigenvectors: "<<std::endl<< A_IK <<std::endl; );

        // Coordinate frame of OOBB
        if(A_IK(2,2) <= 0){ // if z-Axis is negativ -> *-1
            A_IK.col(1).swap(A_IK.col(0));
            A_IK.col(2) *= -1.0;
        }
        LOGTBLEVEL3( m_pSimulationLog, "---> Coordinate Frame: "<<std::endl<< A_IK <<std::endl; );



    }
public:

    template<typename TParser>
    std::tuple< std::unique_ptr<typename TParser::SettingsModuleType >,
        std::unique_ptr<typename TParser::ExternalForcesModuleType >,
        std::unique_ptr<typename TParser::ContactParamModuleType>,
        std::unique_ptr<typename TParser::InitStatesModuleType >,
        std::unique_ptr<typename TParser::BodyModuleType >,
        std::unique_ptr<typename TParser::GeometryModuleType >,
        std::unique_ptr<typename TParser::VisModuleType>,
        std::unique_ptr<typename TParser::MPIModuleType>
        >
    createParserModules(TParser * p) {

        using SettingsModuleType       = typename TParser::SettingsModuleType ;
        using ContactParamModuleType   = typename TParser::ContactParamModuleType;
        using GeometryModuleType       = typename TParser::GeometryModuleType ;
        using InitStatesModuleType     = typename TParser::InitStatesModuleType ;
        using ExternalForcesModuleType = typename TParser::ExternalForcesModuleType ;
        using BodyModuleType           = typename TParser::BodyModuleType ;
        using VisModuleType            = typename TParser::VisModuleType ;
        using MPIModuleType            = typename TParser::MPIModuleType ;

        auto sett = std::unique_ptr<SettingsModuleType >(nullptr);
        auto geom = std::unique_ptr<GeometryModuleType >(nullptr);
        auto vis = std::unique_ptr<VisModuleType>(nullptr);
        auto es  = std::unique_ptr<ExternalForcesModuleType >(nullptr);
        auto con = std::unique_ptr<ContactParamModuleType>(nullptr);
        auto mpi = std::unique_ptr<MPIModuleType>( nullptr );

        auto is  = std::unique_ptr<InitStatesModuleType >(new InitStatesModuleType(p, &m_initStates,nullptr ));
        auto bm  = std::unique_ptr<BodyModuleType>(new BodyModuleType(p,  nullptr , is.get(), nullptr , nullptr , nullptr )) ;

        return std::make_tuple(std::move(sett),std::move(es),std::move(con),std::move(is),std::move(bm),std::move(geom),std::move(vis),std::move(mpi));
    }


private:



    Logging::Log * m_pSimulationLog;

    boost::filesystem::path m_sceneFilePath;

    Vector3 m_r_G_loc;       ///< Local Geometric center of all point masses
    Vector6 m_I_theta_loc;   ///< Local Intertia tensor (binet (euler) inertia tensor [a_00,a_01,a_02,a_11,a_12,a_22] ) in intertial frame I
    AABB m_aabb_loc;         ///< Local AABB of point masses

    // GLOBAL STUFF =================================================================
    struct MassPoint {
        MassPoint(const RigidBodyState & s, unsigned int nPoints = 0): m_state(s), m_points(nPoints+1){
            m_points[0] =  s.getPosition();
        }
        std::vector<Vector3> m_points; //< For the prediction
        RigidBodyState m_state; // Init Condition of the MassPoint
    };
    const unsigned int m_nPointsPredictor = 0;
    const PREC m_deltaT = 0.1;

    typename DynamicsSystemType::RigidBodyStatesContainerType m_initStates;
    typename std::unordered_map<RankIdType,std::vector<RigidBodyIdType> > m_bodiesPerRank;

    AABB    m_aabb_glo;        ///< Global AABB of point masses
    Vector3 m_r_G_glo;         ///< Global geometric center ("G") of all point masses
    Vector6 m_I_theta_G_glo;   ///< Global intertia tensor ( in center point G) (binet (euler) inertia tensor [a_00,a_01,a_02,a_11,a_12,a_22] ) in intertial frame I

    std::unordered_map<RankIdType,AABB> m_rankAABBs;

    std::unordered_map<RigidBodyIdType,MassPoint> m_massPoints_glo; ///< local mass points

    unsigned int m_nGlobalSimBodies; ///< Only for a check with MassPoints
    GridBuilderSettings m_settings;
};

}; //MPILayer


#endif

