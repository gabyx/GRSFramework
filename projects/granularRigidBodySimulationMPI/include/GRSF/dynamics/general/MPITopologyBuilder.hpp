// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_general_MPITopologyBuilder_hpp
#define GRSF_dynamics_general_MPITopologyBuilder_hpp

#include <mpi.h>

#include <pugixml.hpp>

#include <functional>
#include <numeric>

#include "GRSF/common/AssertionDebug.hpp"
#include "GRSF/common/TypeDefs.hpp"
#include "GRSF/common/LogDefines.hpp"

#include "GRSF/common/ApplicationCLOptions.hpp"
#include "GRSF/singeltons/FileManager.hpp"

#include "GRSF/dynamics/general/MPIMessages.hpp"
#include "GRSF/dynamics/general/MPICommunication.hpp"

#include "GRSF/systems/SceneParserMPI.hpp"
#include "GRSF/systems/SceneParserModulesCreatorTB.hpp"

#include DynamicsSystem_INCLUDE_FILE
#include "GRSF/dynamics/collision/Collider.hpp"

#include "GRSF/dynamics/general/MPITopologyBuilderSettings.hpp"

#include "GRSF/dynamics/collision/geometry/AABB.hpp"
#include "GRSF/dynamics/collision/geometry/OOBB.hpp"

#include "GRSF/dynamics/general/KdTree.hpp"
#include "ApproxMVBB/ComputeApproxMVBB.hpp"


namespace MPILayer {


    template<typename TProcCommunicator>
    class TopologyBuilder {

    public:

        DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
        DEFINE_MPI_INFORMATION_CONFIG_TYPES

        using TopologyBuilderEnumType = TopologyBuilderEnum;
        using ProcessCommunicatorType = TProcCommunicator;

        using RebuildSettingsType     = TopologyBuilderSettings::RebuildSettings;

        TopologyBuilder(TopologyBuilderEnumType type,
                std::shared_ptr<DynamicsSystemType> pDynSys,
                std::shared_ptr<ProcessCommunicatorType > pProcCommunicator,
                RebuildSettingsType rebuildSettings)
            : m_type(type),  m_rebuildSettings(rebuildSettings), m_pDynSys(pDynSys), m_pProcCommunicator(pProcCommunicator),
              m_pSimulationLog(nullptr) {
            m_pSimulationLog = Logging::LogManager::getSingleton().getLog("SimulationLog");
            GRSF_ASSERTMSG(m_pSimulationLog, "There is no SimulationLog in the LogManager!");
        }

        bool checkAndRebuild(unsigned int timeStep, PREC currentTime) {
            // Check each X timestep
            if(  m_rebuildSettings.m_mode == RebuildSettingsType::Mode::DYNAMIC &&
                    timeStep!= 0 &&
                    (timeStep % m_rebuildSettings.m_policyCheckEachXTimeStep) == 0
              ) {

                if(m_rebuildSettings.m_policy == RebuildSettingsType::Policy::ALWAYS_REBUILD) {

                    this->rebuildTopology(currentTime);
                    return true;

                } else if(m_rebuildSettings.m_policy == RebuildSettingsType::Policy::BODY_LIMIT) {

                    bool rebuild = false;
                    if( m_pDynSys->m_simBodies.size() >= m_rebuildSettings.m_bodyLimit) {
                        rebuild = true;
                    }
                    // Agree on all processes to rebuild!
                    rebuild = communicateRebuild(rebuild);

                    if(rebuild) {
                        this->rebuildTopology(currentTime);
                        return true;
                    }

                }
            }
            return false;
        };

        void initLogs(boost::filesystem::path localSimFolderPath) {
            m_localSimFolderPath = localSimFolderPath;
        }

        virtual void initTopology(PREC currentTime = 0) = 0;
        virtual void rebuildTopology(PREC currentTime) = 0;

        virtual ~TopologyBuilder() {}

    protected:

        bool communicateRebuild(bool rebuild) {
            // Synchronize rebuilding over all processes by communication
            LOGTBLEVEL1(m_pSimulationLog,"MPI> Gather topology rebuild flags: "<<std::endl;)
            std::vector<char> rebuildGather;
            char r = rebuild? 1 : 0;
            m_pProcCommunicator->allGather( r,rebuildGather, MPILayer::MPICommunicatorId::SIM_COMM);

            if( TOPOBUILDER_LOGLEVEL > 2) {
                Utilities::printVectorNoCopy(*m_pSimulationLog,rebuildGather.begin(),rebuildGather.end(),",");
            }

            // Sum up all boolean values and decide if all neighbours rebuild?
            unsigned int n = std::accumulate(rebuildGather.begin(), rebuildGather.end(), 0, std::plus<char>() );
            LOGTBLEVEL1(m_pSimulationLog,"MPI> n="<<n<<" processes want to rebuild!"<<std::endl;)
            if(n>0) { // if one or more process wants to rebuild then rebuild!
                LOGTBLEVEL1(m_pSimulationLog,"MPI> Rebuild accepted!"<<std::endl;)
                return true;
            }
            LOGTBLEVEL1(m_pSimulationLog,"MPI> Rebuild not accepted!"<<std::endl;)
            return false;
        }


        PREC m_currentTime;
        unsigned int m_builtTopologies = 0;

        const TopologyBuilderEnumType   m_type;
        const RebuildSettingsType       m_rebuildSettings;

        std::shared_ptr<DynamicsSystemType>      m_pDynSys;
        std::shared_ptr<ProcessCommunicatorType> m_pProcCommunicator;

        Logging::Log * m_pSimulationLog;
        boost::filesystem::path m_localSimFolderPath;
    };


#define DEFINE_TOPOLOGYBUILDER_BASE_TYPES( _Base_ ) \
public: \
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES  \
    DEFINE_MPI_INFORMATION_CONFIG_TYPES \
    using RigidBodyStatesContainerType = typename DynamicsSystemType::RigidBodyStatesContainerType; \
    using RigidBodySimContainerType    = typename DynamicsSystemType::RigidBodySimContainerType ; \
    using RigidBodyStaticContainerType = typename DynamicsSystemType::RigidBodyStaticContainerType ; \
    \
    using TopologyBuilderEnumType = typename _Base_::TopologyBuilderEnumType; \
    using ProcessCommunicatorType = typename _Base_::ProcessCommunicatorType; \
    \
protected: \
    using RebuildSettingsType       = typename _Base_::RebuildSettingsType;    \
    using _Base_::m_pSimulationLog; \
    using _Base_::m_pDynSys; \
    using _Base_::m_pProcCommunicator; \
    using _Base_::m_currentTime; \
    using _Base_::m_builtTopologies; \
    using _Base_::m_localSimFolderPath; \
    using _Base_::m_type; \
    using _Base_::m_rebuildSettings; \

    class MassPointPrediction {
    public:

        DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

        using MassPointPredSettingsType = TopologyBuilderSettings::MassPointPredSettings;

        using Vector3Vec = StdVecAligned<Vector3>;

        struct MassPoint {

            using StateType = RigidBodyState;

            MassPoint(const StateType * s): m_state(s) {}

            Vector3Vec * m_points = nullptr;
            unsigned int m_pointIdx = 0;
            unsigned int m_numPoints = 0;

            const StateType * m_state = nullptr; ///< Init Condition of the MassPoint pointing to m_initStates;

            bool m_isOutlier = false;
        };
        using MassPointType = MassPoint;

        template<typename Container>
        inline void reserve(Container & c,
                unsigned int n) {
            c.reserve(n*m_massPointPredSettings.m_nPoints);
        }

        inline MassPointPredSettingsType & getSettings() {
            return m_massPointPredSettings;
        };

    protected:
        MassPointPredSettingsType m_massPointPredSettings;

    public:
        MassPointPrediction(const MassPointPredSettingsType & settings): m_massPointPredSettings(settings) {}

        /** Predict Bodies over some steps
         * Either use Fluidix to quickly approximate a simulation or
         * do a simple integration with only gravity of all point masses, stop if collided with non simulated bodies
         */
        template<typename TExternalForceList, typename TRigidBodyContainer, bool predictOutliers = false>
        void predictMassPoints(std::vector<MassPointType> & massPoints,
                const TExternalForceList & forceList,
                const TRigidBodyContainer & staticBodies,
                Vector3Vec & predictedPoints) {


            ColliderPoint pointCollider;
            PREC dT = m_massPointPredSettings.m_deltaT;
            unsigned int nPoints = m_massPointPredSettings.m_nPoints;

            auto * gravityField = forceList.getGravityField();
            Vector3 gravity = Vector3::Zero();
            if( gravityField ) {
                gravity= gravityField->getGravity();
            }

            predictedPoints.reserve(predictedPoints.size() + massPoints.size()*nPoints);

            for(auto & massPoint : massPoints) {

                GRSF_ASSERTMSG(massPoint.m_state,"State null");

                // outliers are not predicted
                // since they are not used in further topology computations
                if( !predictOutliers && massPoint.m_isOutlier) {
                    continue;
                }

                // add initial state
                massPoint.m_points =  &predictedPoints;
                massPoint.m_pointIdx = predictedPoints.size();  // link index of the predicted points
                massPoint.m_numPoints = 1; // set number of points
                predictedPoints.push_back(massPoint.m_state->getPosition());


                Vector3 vel = massPoint.m_state->getVelocityTrans();

                // Predict mass points with gravity, no collision detection so far with static objects
                // r_s(t) = 1/2*g*t^2 + v * t + r_s
                for(unsigned int i = 0; i <nPoints; i++ ) {

                    predictedPoints.push_back( predictedPoints.back() + dT * vel + 0.5*gravity*dT*dT );
                    vel +=  gravity*dT;

                    massPoint.m_numPoints += 1;

                    // intersect with all static bodies, if intersection abort, point is proxed onto body!
                    bool hitObject = false;
                    for(auto & pBody : staticBodies) {
                        if( pointCollider.intersectAndProx(pBody, predictedPoints.back() ) ) {
                            hitObject = true;
                            // dont break, prox to all static objects, (corners are tricky!)
                        }
                    }

                    if(hitObject) {
                        break;
                    }

                }
                //
                //            LOGTBLEVEL3(m_pSimulationLog, "---> TopoBuilder: body: "
                //            <<RigidBodyId::getBodyIdString(massPoint.m_state->m_id) << " last prediction: " << predictedPoints.back().transpose()  <<std::endl);
            }

            //        LOGTBLEVEL2(m_pSimulationLog, "---> TopoBuilder: Predicted mass points: " << predictedPoints.size() << std::endl;);
        }

    };


#define DEFINE_MASSPOINTPREDICTION_TYPES \
    using MassPointPredSettingsType = typename MassPointPrediction::MassPointPredSettingsType; \
    using MassPointType = typename MassPointPrediction::MassPoint; \
    using Vector3Vec = typename MassPointPrediction::Vector3Vec; \

    class OutlierFilter {
    public:
        DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

        using OutlierFilterSettingsType = TopologyBuilderSettings::OutlierFilterSettings;
        using MassPointType = typename MassPointPrediction::MassPointType;





    private:

        using PointType = VectorBStat< const typename MassPointType::StateType::DisplacementType,  3>;
        struct PointGetter {

            static const PointType get(const MassPointType * p) {
                return p->m_state->m_q.head<3>();
            }
        };

    public:

        using DistCompTraits  = KdTree::DefaultDistanceCompTraits<Vector3,PointGetter>;
        using PointDataTraits = KdTree::DefaultPointDataTraits<3,
                                                                  PointType,
                                                                  MassPointType *,
                                                                  PointGetter,
                                                                  DistCompTraits >;
        using PointListType   = typename PointDataTraits::PointListType;
        using Filter          = KdTree::NearestNeighbourFilter<PointDataTraits>;

    private:
        OutlierFilterSettingsType m_settings;
        Filter m_filter;

    public:

        OutlierFilter(const OutlierFilterSettingsType & f)
            :  m_settings(f){}

        bool isEnabled() {
            return m_settings.m_enabled;
        }

        using NearestDistancesType = Filter::NearestDistancesType;
        inline NearestDistancesType & getNearestDists(){
            return m_filter.getNearestDists();
        }

        auto getMoments() -> decltype(m_filter.getMoments()) {
            return m_filter.getMoments();
        }

        auto getSettings() -> decltype(m_filter.getSettings()) {
            return m_filter.getSettings();
        }

        std::size_t filter(std::vector<MassPointType> & massPoints,
                const AABB3d & aabb) {


            // gets modified by filter function
            PointListType input(massPoints.size());
            unsigned int i=0;
            for(auto & p : massPoints) {
                input[i] = &p;
                ++i;
            }

            PointListType output;

            // Get all outliers
            m_filter.setSettings(m_settings.m_kNNMean, m_settings.m_stdDevMult, m_settings.m_allowSplitAbove);
            m_filter.filter(input,aabb,output,true);

            // Mark all points as outliers
            for(auto p: output) {
                p->m_isOutlier = true;
            }
            return output.size();
        }


    };

#define DEFINE_OUTLIERFILTER_TYPES \
    using OutlierFilterSettingsType = typename OutlierFilter::OutlierFilterSettingsType; \


    template<typename TProcCommunicator, typename TSetting >
    class GridTopologyBuilderBase : public TopologyBuilder<TProcCommunicator> {
    public:
        using TopoBase = TopologyBuilder<TProcCommunicator>;
        DEFINE_TOPOLOGYBUILDER_BASE_TYPES(TopoBase)

    public:
        using SettingsType = TSetting;
        DEFINE_MASSPOINTPREDICTION_TYPES
        DEFINE_OUTLIERFILTER_TYPES
        /** TimeStepper Settings which need to be communicated */
        struct TimeStepperSettings {
            PREC m_startTime = 0;
        };
        using TimeStepperSettingsType = TimeStepperSettings;

    protected:



        friend class TopologyBuilderMessageWrapperOrientation<GridTopologyBuilderBase>;
        friend class TopologyBuilderMessageWrapperBodies<GridTopologyBuilderBase>;

        /** Messages for sending/receiving bodies and solving extent collaboratively (Binet Tensor) */
        TopologyBuilderMessageWrapperOrientation<GridTopologyBuilderBase>   m_messageOrient;
        TopologyBuilderMessageWrapperBodies<GridTopologyBuilderBase>        m_messageBodies;

        /** Scene file currently simulated */
        boost::filesystem::path m_sceneFilePath; ///< The parsed scene file for the initial topology

        /** Grid Settings for the building process*/
        const SettingsType m_settings;

        /** LOCAL STUFF (non-master ranks) ============================================= */

//        AABB3d m_I_aabb_loc;     ///< Local AABB of point masses
//        std::vector<MassPointType> m_massPoints_loc; ///< local mass points
//        Vector3Vec m_predPoints_loc; /// point cloud of all predicted mass points
//        unsigned int m_countPoints = 0;

        /** Local computations can only be done for certain modes, see determineLocalComputations() */
        bool m_doComputations_loc = true;
        /** ============================================================================ */

        /** GLOBAL STUFF (master rank only) ============================================ */
        typename std::unordered_map<RankIdType, std::vector<const RigidBodyState *> > m_bodiesPerRank; ///< rank to all pointers in m_initStates;


        std::unordered_map<RankIdType,AABB3d> m_rankAABBs;

        /** BOTH (master and other ranks) ============================================== */
        std::vector<MassPointType> m_massPoints;     ///< global/local mass points
        Vector3Vec m_predPoints;                     /// point cloud of all predicted mass points
        unsigned int m_countPoints = 0;

        // Accumulators for Binet Tensor
        Vector3 m_r_G;           ///< Local Geometric center of all point masses
        Vector6 m_I_theta_G;     ///< Local intertia tensor (binet tensor in center I) [ a_00,a_01,a_02,a_11,a_12,a_22] ) in intertial frame I

        //Final Bounding Box on master
        bool m_aligned;         ///< aligned = AABB, otherwise OOBB
        AABB3d  m_aabb;         ///< Min/Max of OOBB fit around points in frame K (OOBB) or I (AABB),
        Matrix33 m_A_IK;        ///< Transformation of OOBB cordinate frame K (oriented bounding box of point cloud) to intertia fram I


        // references into this container remain valid, unordered_map, even if rehash
        RigidBodyStatesContainerType m_initStates; ///< All states received from all processes
        /** ============================================================================ */


        TimeStepperSettingsType m_timeStepperSettings;

        MassPointPrediction m_massPointPrediction;
        OutlierFilter       m_globalOutlierFilter;


    protected:


        template<typename... T>
        GridTopologyBuilderBase(const SettingsType & setting,
                const MassPointPredSettingsType & massSettings,
                const OutlierFilterSettingsType  & globalFilterSettings,
                boost::filesystem::path sceneFile,
                T &&... t
                               )
            : TopoBase(std::forward<T>(t)...),
              m_messageOrient(this),
              m_messageBodies(this) ,
              m_sceneFilePath(sceneFile), m_settings(setting),
               m_massPointPrediction(massSettings),
              m_globalOutlierFilter(globalFilterSettings)
              {

        }

        /** Compute Minimum Bounding Box with ApproxMVBB source code */
        void makeMVBB(const Vector3Vec & I_points, Matrix33 & A_IK, AABB3d & K_aabb ) {

            LOGTBLEVEL3(m_pSimulationLog, "Points MVBB \n:");
            K_aabb.reset(); // missuse for bounding box computation for precision of the mvbb code
            for(auto & p: I_points) {
                K_aabb.unite(p);
                LOGTBLEVEL3(m_pSimulationLog, p.transpose().format(MyMatrixIOFormat::SpaceSep) << "\n");
            }
            LOGTBLEVEL3(m_pSimulationLog, std::endl;);

            // Map I_points in std::vector to matrix
            if(sizeof(Vector3) != 3*sizeof(PREC)) {
                GRSF_ERRORMSG("There is padding inside Vector3 which is not allowed here")
            }

            MatrixMap<const ApproxMVBB::Matrix3Dyn> p(I_points.data()->data(), 3, I_points.size());

            PREC epsilon = 0.01*K_aabb.extent().matrix().norm(); // percentage of the approximate extent of the point cloud
            OOBB oobb = ApproxMVBB::approximateMVBB(p,epsilon,400,5,0,6);

            // Set A_IK
            A_IK = oobb.m_q_KI.matrix();

            // set min/max in K frame
            K_aabb = AABB3d(oobb.m_minPoint,oobb.m_maxPoint);

            if(K_aabb.isEmpty()) {
                GRSF_ERRORMSG("Box is empty")
            }
            LOGTBLEVEL3( m_pSimulationLog, "---> GridTopoBuilder: Coordinate Frame: "<<std::endl<< A_IK <<std::endl; );
            LOGTBLEVEL3( m_pSimulationLog, "---> GridTopoBuilder: A_IK * A_IK^T: "<<std::endl<< A_IK*A_IK.transpose() <<std::endl; );
        }

        template<bool aligned = false, bool includePoints=true, bool includeState = false>
        void computeExtent(const std::vector<MassPointType> & massPoints,
                            AABB3d & aabb,
                            const Matrix33 & A_IK = Matrix33::Identity()) {
            LOGTBLEVEL3(m_pSimulationLog, "---> TopoBuilder: Compute extent ..." << std::endl;);
            // Calculate min max
            aabb.reset();
            Vector3 t;
            for(auto & massPoint : massPoints) {

                if(includeState){
                    if(aligned) { // ignore transformation!
                        aabb += massPoint.m_state->getPosition();
                    } else {
                        aabb += A_IK.transpose() * massPoint.m_state->getPosition();
                    }
                }

                // loop over all prediction points of this mass point
                if(includePoints){
                    for(unsigned int i = massPoint.m_pointIdx ; i < massPoint.m_pointIdx + massPoint.m_numPoints; ++i) {

                        if(aligned) { // ignore transformation!
                            aabb += (*massPoint.m_points)[i];
                        } else {
                            t = A_IK.transpose() * (*massPoint.m_points)[i];
                            aabb += t;
                        }
                    }
                }
            }
        }

        template<typename C>
        void buildCenterPoint( const C & points, Vector3 & center) {
            LOGTBLEVEL3(m_pSimulationLog, "---> GridTopoBuilderBase: Build center point ..."<< std::endl;);
            center.setZero();
            for(auto & point : points) {
                center += point;
            }
            if(points.size()>0) {
                center /= points.size();
            }
        }


        template<bool shift = true, typename C>
        void buildBinetTensor(  const C & points,
                                Vector6 & I_theta_O,
                                const Vector3 & I_shift_OG = Vector3::Zero())
        {
            LOGTBLEVEL3(m_pSimulationLog, "---> TopoBuilder: Build Binet Tensor ..."<< std::endl;);
            I_theta_O.setZero();
            // I_r_OS: denotes distance from O to point P, expressed in I system
            for(auto & I_r_OS : points) {
                    // Binet Tensor (expressed in the frame I of the massPoints, at reference point O)
                    // (mass=1, calculate only the 6 essential values)
                    I_theta_O(0) += I_r_OS(0)*I_r_OS(0);
                    I_theta_O(1) += I_r_OS(0)*I_r_OS(1);
                    I_theta_O(2) += I_r_OS(0)*I_r_OS(2);
                    I_theta_O(3) += I_r_OS(1)*I_r_OS(1);
                    I_theta_O(4) += I_r_OS(1)*I_r_OS(2);
                    I_theta_O(5) += I_r_OS(2)*I_r_OS(2);
            }
            if(shift) {
                shiftBinetTensor(I_theta_O, I_shift_OG, points.size() );
            }
        }
        /** Shift I_theta_O to I_theta_G (in/out) */
        void shiftBinetTensor(  Vector6 & I_theta_G,
                                const Vector3 & I_r_OG,
                                PREC scalar) {
            // Move intertia tensor to center G
            I_theta_G(0) -= I_r_OG(0)*I_r_OG(0)  * scalar;
            I_theta_G(1) -= I_r_OG(0)*I_r_OG(1)  * scalar;
            I_theta_G(2) -= I_r_OG(0)*I_r_OG(2)  * scalar;
            I_theta_G(3) -= I_r_OG(1)*I_r_OG(1)  * scalar;
            I_theta_G(4) -= I_r_OG(1)*I_r_OG(2)  * scalar;
            I_theta_G(5) -= I_r_OG(2)*I_r_OG(2)  * scalar;
        }

        void solveOrientation(Matrix33 & A_IK, const Vector6 & theta_G) {
            //Calculate principal axis of Theta_G
            Matrix33 Theta_G;
            MatrixHelpers::setSymMatrix(Theta_G,theta_G);


            LOGTBLEVEL3( m_pSimulationLog, "---> GridTopoBuilderBase: Global Binet inertia tensor: " << std::endl
                          << Theta_G << std::endl)

            typename MyMatrixDecomposition::template EigenSolverSelfAdjoint<Matrix33> eigenSolv(Theta_G);

            A_IK = eigenSolv.eigenvectors();
            LOGTBLEVEL3( m_pSimulationLog, "---> GridTopoBuilderBase: Eigenvalues: "<<std::endl<< eigenSolv.eigenvalues().transpose() <<std::endl; );
            LOGTBLEVEL3( m_pSimulationLog, "---> GridTopoBuilderBase: Eigenvectors: "<<std::endl<< A_IK <<std::endl; );

            //        // Correct A_IK to be as orthogonal as possible, project x onto x-y plane, (for no use so far)
            //        A_IK.col(2).normalize();
            //        A_IK.col(0) -= A_IK.col(2).dot(A_IK.col(0)) * A_IK.col(2); A_IK.col(0).normalize();
            //        A_IK.col(1) = A_IK.col(2).cross(A_IK.col(0));              A_IK.col(1).normalize();

            // Coordinate frame of OOBB
            if(A_IK(2,2) <= 0) { // if z-Axis is negativ -> *-1
                A_IK.col(1).swap(A_IK.col(0));
                A_IK.col(2) *= -1.0;
            }
            LOGTBLEVEL3( m_pSimulationLog, "--->GridTopoBuilderBase: Coordinate Frame: "<<std::endl<< A_IK <<std::endl; );
            LOGTBLEVEL3( m_pSimulationLog, "--->GridTopoBuilderBase: x*y,y*z,z*x: "<<
                    A_IK.col(0).dot(A_IK.col(1)) <<", "<<
                    A_IK.col(1).dot(A_IK.col(2)) <<", "<<
                    A_IK.col(2).dot(A_IK.col(0))  <<std::endl; );
        }

        template<bool isMaster>
        void collaborativeSolveExtent(RankIdType masterRank, AABB3d & aabb) {

            LOGTBLEVEL3(m_pSimulationLog, "---> GridTopoBuilderBase: Solve extent collaborative ");
            // Get orientation
            if(isMaster) {
                // broadcast orientation
                m_pProcCommunicator->sendBroadcast(m_messageOrient);
            } else {
                m_pProcCommunicator->receiveBroadcast(m_messageOrient,masterRank);
            }

            LOGTBLEVEL3(m_pSimulationLog, "---> GridTopoBuilderBase: Compute extent locally ");
            AABB3d reduceValue;
            this->computeExtent(m_massPoints, reduceValue, m_A_IK);

            LOGTBLEVEL3(m_pSimulationLog, "---> GridTopoBuilderBase: Do reduction with local extents");
            // Do a reduction of min and max values of the AABBs in K frame
            if(isMaster) { // ROOT
                m_pProcCommunicator->reduce(reduceValue.m_minPoint, aabb.m_minPoint, MPILayer::ReduceFunctions::MinVector3);
                m_pProcCommunicator->reduce(reduceValue.m_maxPoint, aabb.m_maxPoint, MPILayer::ReduceFunctions::MaxVector3);
            } else { //All others
                m_pProcCommunicator->reduce(reduceValue.m_minPoint, MPILayer::ReduceFunctions::MinVector3, masterRank);
                m_pProcCommunicator->reduce(reduceValue.m_maxPoint, MPILayer::ReduceFunctions::MaxVector3, masterRank);
            }

        }


        void filterPoints(std::vector<MassPointType> & points,
                          AABB3d & aabb)
        {
            LOGTBLEVEL1(m_pSimulationLog, "---> GridTopoBuilderBase: Filter point cloud ..." << std::endl);

            LOGTBLEVEL2(m_pSimulationLog, "---> GridTopoBuilderBase: Filter: compute extent ..." << std::endl);
            this->computeExtent<true,false,true>(points,aabb);

            LOGTBLEVEL2(m_pSimulationLog, "---> GridTopoBuilderBase: Filter: outlier filtering ..." << std::endl);
            auto nOutlier = m_globalOutlierFilter.filter(points,aabb);
            LOGTBLEVEL1(m_pSimulationLog, "---> GridTopoBuilderBase: Classified " << nOutlier << " outliers in point cloud." << std::endl);
            if(nOutlier == points.size()) {
                GRSF_ERRORMSG("Filtered all points away! This should not happend! Adjust filter settings!")
            }
        }

        inline void cleanUpLocal() {
            m_initStates.clear();
            m_predPoints.clear();
            m_massPoints.clear();
            m_countPoints = 0;
        }
        inline void cleanUpGlobal() {

            m_initStates.clear();
            m_predPoints.clear();
            m_massPoints.clear();
            m_countPoints = 0;

            m_rankAABBs.clear();
            m_bodiesPerRank.clear();
        }

        void initMakeBoundingBoxGlobal(RankIdType masterRank) {

            // do filtering of point cloud
            if(m_globalOutlierFilter.isEnabled()) {
                filterPoints(m_massPoints,m_aabb);
            }else{
                LOGTBLEVEL1(m_pSimulationLog, "---> GridTopoBuilderBase: Global outlier filter not enabled!" << std::endl);
            }

            if(m_settings.m_buildMode == SettingsType::BuildMode::PREDEFINED) {
                // build a static grid, set m_aabb to settings from scene file.
                if(m_settings.m_aligned) {
                    m_aabb = m_settings.m_aabb;
                    m_A_IK.setIdentity();
                    m_aligned = true;
                } else {
                    m_aabb = m_settings.m_aabb;
                    m_A_IK = m_settings.m_A_IK;
                    m_aligned = false;
                }
            } else if(m_settings.m_buildMode == SettingsType::BuildMode::ALIGNED) {
                m_A_IK.setIdentity();
                m_predPoints.clear();
                LOGTBLEVEL1(m_pSimulationLog, "---> GridTopoBuilderBase:mass points glo: " << m_massPoints.size() << std::endl);
                m_massPointPrediction.predictMassPoints(m_massPoints,m_pDynSys->m_externalForces, m_pDynSys->m_staticBodies, m_predPoints );
                LOGTBLEVEL1(m_pSimulationLog, "---> GridTopoBuilderBase:mass points glo pred: " << m_predPoints.size() << std::endl);
                this->template computeExtent<true>(m_massPoints,m_aabb);
                m_aligned = true;

            } else if(m_settings.m_buildMode == SettingsType::BuildMode::BINET_TENSOR) {

                // Process for Grid Building with Binet Inertia Tensor  =================================================
                m_predPoints.clear();
                m_massPointPrediction.predictMassPoints(m_massPoints,m_pDynSys->m_externalForces, m_pDynSys->m_staticBodies, m_predPoints );
                this->buildCenterPoint(m_predPoints,m_r_G);
                this->template buildBinetTensor(m_predPoints,m_I_theta_G, m_r_G);
                this->solveOrientation(m_A_IK,m_I_theta_G);
                this->template computeExtent(m_massPoints,m_aabb,m_A_IK);
                m_aligned = false;

            } else if(m_settings.m_buildMode == SettingsType::BuildMode::MVBB) {

                m_predPoints.clear();
                m_massPointPrediction.predictMassPoints(m_massPoints,m_pDynSys->m_externalForces, m_pDynSys->m_staticBodies, m_predPoints );
                this->makeMVBB(m_predPoints, m_A_IK, m_aabb);
                m_aligned = false;


            } else {
                GRSF_ERRORMSG("Built type: " << EnumConversion::toIntegral(m_settings.m_buildMode) << " not supported! ")
            }
        }

        void rebuildMakeBoundingBoxGlobal( RankIdType masterRank) {
            // only master rank!

            // up to now
            // m_massPoints has been filled from all ranks as well as m_initStates

            // if we did not do any local computation
            // we filter/predict all points now
            if(!m_doComputations_loc) {
                // filter points
                if(m_globalOutlierFilter.isEnabled()) {
                    filterPoints(m_massPoints,m_aabb);
                }
                // points are marked in m_massPoints!

                // predict points but not outliers (default)
                m_predPoints.clear();
                m_massPointPrediction.predictMassPoints(m_massPoints,
                        m_pDynSys->m_externalForces,
                        m_pDynSys->m_staticBodies,
                        m_predPoints);
            }

            // Finish computation according to build mode
            if(m_settings.m_buildMode == SettingsType::BuildMode::ALIGNED) {

                if(!m_doComputations_loc) {
                    this->template computeExtent<true>(m_massPoints,m_aabb);
                }else{

                }

                m_A_IK.setIdentity();
                m_aligned = true;

            } else if(m_settings.m_buildMode == SettingsType::BuildMode::BINET_TENSOR) {

                if(m_doComputations_loc) {

                    // accummulators have been filled, finish by dividing
                    // m_r_G is now summed
                    // m_countPoints is summed to (how many points we summed for center/theta)
                    m_r_G /= m_countPoints;
                    // m_I_theta_G is now summed!

                    // Shift global to center!
                    this->shiftBinetTensor(m_I_theta_G,m_r_G,m_countPoints);
                    this->solveOrientation(m_A_IK,m_I_theta_G);
                    this->collaborativeSolveExtent<true>(masterRank, m_aabb);
                } else {
                    this->buildCenterPoint(m_predPoints,m_r_G);
                    this->template buildBinetTensor(m_predPoints,m_I_theta_G,m_r_G);
                    this->solveOrientation(m_A_IK,m_I_theta_G);
                    this->template computeExtent(m_massPoints,m_aabb,m_A_IK);
                }
                m_aligned = false;

            } else if(m_settings.m_buildMode == SettingsType::BuildMode::MVBB) {

                this->makeMVBB(m_predPoints, m_A_IK, m_aabb);
                m_aligned = false;
            }
        }

        bool determineLocalComputation(){

            // if globalOutlierFilter is on, then we cannot locally compute the stuff,
            // everything has to be done on the master (since local outlier fitering is not a good idea!)
            // we do global outlier filter on the master and then compute the grid
            // Local computations can only be done for ALIGNED mode and for the BINET_TENSOR mode
            // MVBB has no local computation stage!
            // this->m_rebuildSettings.m_doLocalComputations is set in constructor!

            return  (
                     m_settings.m_buildMode == SettingsType::BuildMode::ALIGNED ||
                     m_settings.m_buildMode == SettingsType::BuildMode::BINET_TENSOR
                     )
                    &&
                    (!m_globalOutlierFilter.isEnabled() && m_rebuildSettings.m_doLocalComputations );
        }

        template<bool isMaster>
        void rebuildMakeBoundingBoxLocal_PreSend(RankIdType masterRank) {
            // local computations before sending to master
            // master and all other do this!

            m_doComputations_loc = determineLocalComputation();

            // we copy all body points into the set m_initStates and make some m_massPoints
            m_massPoints.clear();
            m_massPoints.reserve(m_pDynSys->m_simBodies.size());

            m_initStates.clear();
            m_initStates.reserve(m_pDynSys->m_simBodies.size());
            for(auto & body : m_pDynSys->m_simBodies) {
                auto res = m_initStates.emplace(body->m_id,body); // add new state
                m_massPoints.emplace_back( &(res.first->second) ); // pointer stays valid also if rehash of m_initStates
                LOGTBLEVEL3(m_pSimulationLog, "\t\t state: " << RigidBodyId::getBodyIdString(body->m_id )<< " , " <<  res.first->second.m_q.transpose() << std::endl;);
            }


            if( m_doComputations_loc ) {

                if(m_settings.m_buildMode == SettingsType::BuildMode::BINET_TENSOR) {

                    m_predPoints.clear();
                    m_massPointPrediction.predictMassPoints(m_massPoints, m_pDynSys->m_externalForces, m_pDynSys->m_staticBodies, m_predPoints);
                    m_countPoints = m_predPoints.size(); // init accummulator

                    this->buildCenterPoint(m_predPoints,m_r_G);
                    m_r_G *= m_countPoints; // init accumulator

                    this->template buildBinetTensor<false>(m_predPoints,m_I_theta_G); // init accumulator
                    // Move inertia tensor to geometric center G (steiner formula) on root process!
                    // Here it is still corresponding to I frame: m_I_theta_I

                } else if(m_settings.m_buildMode == SettingsType::BuildMode::ALIGNED) {

                    m_predPoints.clear();
                    m_massPointPrediction.predictMassPoints(m_massPoints, m_pDynSys->m_externalForces, m_pDynSys->m_staticBodies, m_predPoints);
                    m_countPoints = m_predPoints.size();
                    this->template computeExtent<true>(m_massPoints,m_aabb); // init accumulator

                    // add our own local computed AABB to the ranksAABB list (this should actually only do root)
                    if(isMaster){
                        m_rankAABBs.emplace(masterRank, m_aabb);
                    }

                } else if(m_settings.m_buildMode == SettingsType::BuildMode::MVBB) {
                    GRSF_ERRORMSG("Local computations should be off for MVVB build mode!")
                } else {
                    GRSF_ERRORMSG("This should not happen! Undefined BuildMode in Rebuilding")
                }
                // otherwise dont do any computation (it might be send anyway but is not used on the master!)
                LOGTBLEVEL1(m_pSimulationLog, "---> GridTopoBuilderBase: Local computations performed! "<<std::endl);
            } else {
                // otherwise dont do any computation (it might be send anyway but is not used on the master!)
                LOGTBLEVEL1(m_pSimulationLog, "---> GridTopoBuilderBase: No local computations performed, because disabled!" <<std::endl);
            }
        }

        void rebuildMakeBoundingBoxLocal_PostSend(RankIdType masterRank) {
            if(m_settings.m_buildMode == SettingsType::BuildMode::BINET_TENSOR && m_doComputations_loc) {
                this->template collaborativeSolveExtent<false>(masterRank,m_aabb);
            }
        }


        using XMLNodeType = pugi::xml_node;
        static const auto  nodePCData = pugi::node_pcdata;

        template<bool writeAABBList = true,
                 bool writePoints = false,
                 bool writeHistogram = false>
        void writeTopoInfo(XMLNodeType & root,
                           PREC currentTime, PREC buildTime)
        {
            std::stringstream ss;
            root.append_attribute("time").set_value( std::to_string(currentTime).c_str() );
            root.append_attribute("builtTime").set_value( std::to_string(buildTime).c_str() );

            XMLNodeType node;

            // Write points
            if(writePoints) {
                ss.str("");
                node = root.append_child("Points");
                for(auto & p : m_predPoints ) {
                    ss << p.transpose().format(MyMatrixIOFormat::SpaceSep) << std::endl;
                }
                node.append_child(nodePCData).set_value( ss.str().c_str() );
            }

            // Write Histogramm
            if(writeHistogram && m_globalOutlierFilter.isEnabled()){
                ss.str("");
                node = root.append_child("NearestDistanceHistogram");
                auto mom = m_globalOutlierFilter.getMoments();

                node.append_attribute("mean").set_value( std::to_string(mom.first).c_str() );
                node.append_attribute("stdDeviation").set_value( std::to_string(mom.second).c_str() );

                auto s= m_globalOutlierFilter.getSettings();
                node.append_attribute("kNNMean").set_value( std::to_string(std::get<0>(s)).c_str() );
                node.append_attribute("stdDevMult").set_value( std::to_string(std::get<1>(s)).c_str() );
                node.append_attribute("allowSplitAbove").set_value( std::to_string(std::get<2>(s)).c_str() );

                for(auto & p : m_globalOutlierFilter.getNearestDists()) {
                    ss << p << " " ;
                }
                node.append_child(nodePCData).set_value( ss.str().c_str() );
            }

            // Write AABB list
            if(writeAABBList){
                node = root.append_child("AABBList");
                ss.str("");
                for(auto & aabbIt : m_rankAABBs) {

                    ss << aabbIt.first
                            << " " << aabbIt.second.m_minPoint.transpose().format(MyMatrixIOFormat::SpaceSep)
                            << " " << aabbIt.second.m_maxPoint.transpose().format(MyMatrixIOFormat::SpaceSep)
                            << std::endl;

                }
                node.append_child(nodePCData).set_value( ss.str().c_str() );
            }
        }
    };

#define DEFINE_TOPOLOGYBUILDERGRID_BASE_TYPES( _Base_ ) \
    \
public: \
    DEFINE_TOPOLOGYBUILDER_BASE_TYPES( _Base_::TopoBase ) \
    DEFINE_MASSPOINTPREDICTION_TYPES \
    DEFINE_OUTLIERFILTER_TYPES \
public: \
    using SettingsType = typename _Base_::SettingsType;\
    using TimeStepperSettingsType = typename _Base_::TimeStepperSettingsType; \
    \
private: \
    using _Base_::m_sceneFilePath;\
    using _Base_::m_settings;\
    /*LOCAL*/\
    using _Base_::m_messageOrient; \
    using _Base_::m_messageBodies; \
    \
    /*GLOBAL*/ \
    using _Base_::m_bodiesPerRank;\
    using _Base_::m_rankAABBs;\
    \
    /*BOTH*/ \
    using _Base_::m_r_G;\
    using _Base_::m_I_theta_G;\
    \
    using _Base_::m_aligned;\
    using _Base_::m_aabb;\
    using _Base_::m_A_IK; \
    \
    using _Base_::m_massPoints;\
    using _Base_::m_predPoints;\
    using _Base_::m_countPoints;\
    using _Base_::m_initStates;\
    \
    using _Base_::m_timeStepperSettings;\
    using _Base_::m_massPointPrediction;\
    using _Base_::m_globalOutlierFilter;

    template<typename TProcCommunicator>
    class GridTopologyBuilder : public GridTopologyBuilderBase<TProcCommunicator,GridBuilderSettings>  {

    public:
        using Base = GridTopologyBuilderBase<TProcCommunicator,GridBuilderSettings>;
        DEFINE_TOPOLOGYBUILDERGRID_BASE_TYPES(Base)

    private:

        friend class ParserModulesCreatorTopoBuilder<GridTopologyBuilder>;
        friend class TopologyBuilderMessageWrapperResults<GridTopologyBuilder>;

        unsigned int m_nGlobalSimBodies = 0; ///< Only for a check with MassPoints

        /** GLOBAL STUFF ================================================================ */
        typename SettingsType::ProcessDimType m_processDim; ///< The current process dim size
        /** ============================================================================= */

        TopologyBuilderMessageWrapperResults<GridTopologyBuilder>   m_messageWrapperResults;

    public:

        GridTopologyBuilder(std::shared_ptr<DynamicsSystemType> pDynSys,
                std::shared_ptr<ProcessCommunicatorType > pProcCommunicator,
                const RebuildSettingsType & rebuildSettings,
                const MassPointPredSettingsType & massPointPredSettings,
                const OutlierFilterSettingsType & globalOutlierFilterSettings,
                const SettingsType & settings,
                boost::filesystem::path sceneFilePath,
                unsigned int nGlobalSimBodies)
            :Base(settings,massPointPredSettings,globalOutlierFilterSettings,sceneFilePath,
                    TopologyBuilderEnumType::GRIDBUILDER, pDynSys, pProcCommunicator,rebuildSettings),
             m_nGlobalSimBodies(nGlobalSimBodies),
             m_messageWrapperResults(this) {
        }

        void initTopology(PREC currentTime = 0) {
            LOGTB(m_pSimulationLog,"---> GridTopoBuilder: initialize Topology" <<std::endl;)

            START_TIMER(startTopoTime)

            // get initial startTime which was parsed in ( gets overwritten by global initial condition if specified to continue from)
            m_timeStepperSettings.m_startTime = m_pDynSys->getSettingsTimeStepper().m_startTime;

            // increment number of built topologies
            m_currentTime = currentTime;
            ++m_builtTopologies;

            RankIdType masterRank = this->m_pProcCommunicator->getMasterRank();

            if(this->m_pProcCommunicator->hasMasterRank()) {

                if(m_settings.m_processDim(0)*m_settings.m_processDim(1)*m_settings.m_processDim(2) != m_pProcCommunicator->getNProcesses()) {
                    GRSF_ERRORMSG("processDim: " << m_settings.m_processDim << " does not fit "<<m_pProcCommunicator->getNProcesses()<<"processes!");
                }

                // Parse all initial condition from the scene file (global initial condition is read too)
                ParserModulesCreatorTopoBuilder<GridTopologyBuilder> c(this);
                SceneParserMPI<GridTopologyBuilder, ParserModulesCreatorTopoBuilder<GridTopologyBuilder>::template SceneParserTraits> parser(c, m_pSimulationLog,
                        ApplicationCLOptions::getSingleton().getMediaDir()) ; // this class is the modules generator

                // clean init states:
                m_initStates.clear();
                parser.parseScene(m_sceneFilePath);

                if(m_initStates.size() != m_nGlobalSimBodies) {
                    GRSF_ERRORMSG("Parsed to little initial states in scene file: " << m_sceneFilePath << " states: " << m_initStates.size() << "globalSimBodies: " << m_nGlobalSimBodies <<std::endl);
                }

                LOGTBLEVEL2(m_pSimulationLog, "---> GridTopoBuilder: parsed states: "<<std::endl;);
                m_massPoints.clear();
                m_massPoints.reserve(m_initStates.size());
                for(auto & s : m_initStates) {
                    // Save mass points
                    m_massPoints.emplace_back( &s.second );
                    LOGTBLEVEL3(m_pSimulationLog, "\t\t state @"<<&s.second <<" : "
                            << RigidBodyId::getBodyIdString(s.second.m_id )<< " , " << s.second.m_q.transpose() << std::endl;);

                }

                this->initMakeBoundingBoxGlobal(masterRank);

                adjustGrid();

                buildTopo();
                sendGrid(masterRank);

                STOP_TIMER_SEC(buildTime, startTopoTime)

                // Write grid data to file
                writeTopo(buildTime);

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
            for(auto & s :  m_pDynSys->m_bodiesInitStates) {
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

            START_TIMER(startTopoTime)

            // increment number of built topologies
            ++m_builtTopologies;
            m_currentTime = currentTime;

            LOGTB(m_pSimulationLog,"---> GridTopoBuilder: Clear all init states in DynamicsSystem" <<std::endl;)
            m_pDynSys->m_bodiesInitStates.clear();



            RankIdType masterRank = this->m_pProcCommunicator->getMasterRank();
            if(this->m_pProcCommunicator->hasMasterRank()) {

                // compute local stuff for the corresponding build method
                this->template rebuildMakeBoundingBoxLocal_PreSend<true>(masterRank);

                // Receive Messages
                // Fill linear array with all ranks except master
                std::vector<RankIdType> ranks;
                unsigned int count=0;
                std::generate_n(std::back_inserter(ranks),
                        this->m_pProcCommunicator->getNProcesses()-1,
                [&]() {
                    if(count == masterRank) {
                        return (++count)++;
                    } else {
                        return count++;
                    }
                });

                m_messageBodies.resetBeforLoad(); // Resets all variables for the appropriate build method in this class
                this->m_pProcCommunicator->receiveMessageFromRanks(m_messageBodies, ranks, m_messageBodies.m_tag);

                //Check if number of init states received is equal to the total in the simulation
                if(m_nGlobalSimBodies != m_initStates.size()) {
                    GRSF_ERRORMSG("m_nGlobalSimBodies: " << m_nGlobalSimBodies << " != " <<  m_initStates.size() <<std::endl);
                }


                this->rebuildMakeBoundingBoxGlobal(masterRank);

                adjustGrid();

                buildTopo();
                sendGrid(masterRank);

                STOP_TIMER_SEC(buildTime, startTopoTime)
                // Write grid data to file
                writeTopo(buildTime);


                cleanUpGlobal();
                cleanUpLocal();

            } else {

                // compute local stuff for the corresponding build method
                this->template rebuildMakeBoundingBoxLocal_PreSend<false>(masterRank);

                this->m_pProcCommunicator->sendMessageToRank(m_messageBodies, masterRank, m_messageBodies.m_tag);

                this->rebuildMakeBoundingBoxLocal_PostSend(masterRank);

                receiveGrid(masterRank);

                buildTopo();
                cleanUpLocal();
            }

            LOGTBLEVEL1(m_pSimulationLog, "---> GridTopoBuilder: States in m_bodiesInitStates: (" << m_pDynSys->m_bodiesInitStates.size()<< ")" <<std::endl;);
            for(auto & s :  m_pDynSys->m_bodiesInitStates) {
                LOGTBLEVEL3(m_pSimulationLog, "\t\t state: "
                        << RigidBodyId::getBodyIdString(s.second.m_id )<< " , q: "
                        << s.second.m_q.transpose() << " u: " << s.second.m_u.transpose() << std::endl;);
            }

            LOGTB(m_pSimulationLog,"---> GridTopoBuilder: rebuild topology finished!" <<std::endl;)

        }

    private:

        void cleanUpGlobal() {
            Base::cleanUpGlobal();
        }

        void cleanUpLocal() {
            Base::cleanUpLocal();
        }


        void adjustGrid() {

            // Copy processDim size from initial settings:
            m_processDim = m_settings.m_processDim;

            Array3 e = m_aabb.extent();

            // Sort process dim according to extent (if needed)
            if(m_settings.m_matchProcessDimToExtent) {
                // Sort procDim ascending
                auto procDim = m_processDim;
                std::sort(procDim.data(),procDim.data() + procDim.size());

                // Get min extent idx
                Array3::Index minIdx;
                e.minCoeff(&minIdx);

                // set min proc dim for min extent
                m_processDim(minIdx) = procDim(0);
                // set proc dim for remaining
                unsigned int i1 = (minIdx+1)%3;
                unsigned int i2 = (minIdx+2)%3;
                if( e(i1) <= e(i2) ) {
                    m_processDim(i1) = procDim(1);
                    m_processDim(i2) = procDim(2);
                } else {
                    m_processDim(i1) = procDim(2);
                    m_processDim(i2) = procDim(1);
                }
            }

            //Adjust box to minimal box size =  procDim * min_gridSize,
            Array3 limits = m_processDim.template cast<PREC>() * m_settings.m_minCellSize;
            m_aabb.expandToMinExtentAbsolute( limits );

        }

        void buildTopo() {

            LOGTBLEVEL1(m_pSimulationLog, "---> GridTopoBuilder: build grid ..." << std::endl;);

            if(m_rebuildSettings.m_mode == RebuildSettingsType::Mode::STATIC) {
                LOGTBLEVEL1( m_pSimulationLog, "---> GridTopoBuilder: Create static ProcessTopologyGrid: " << std::endl;)
            } else {
                LOGTBLEVEL1( m_pSimulationLog, "---> GridTopoBuilder: Create new ProcessTopologyGrid: "<< std::endl;)
            }

            if(  m_settings.m_buildMode == GridBuilderSettings::BuildMode::ALIGNED) {
                LOGTBLEVEL1( m_pSimulationLog,"\t buildMode: ALIGNED" << std::endl);
            } else if(m_settings.m_buildMode == GridBuilderSettings::BuildMode::PREDEFINED) {
                LOGTBLEVEL1( m_pSimulationLog, "\t buildMode: PREDEFINED" << std::endl );
            } else if (m_settings.m_buildMode == GridBuilderSettings::BuildMode::BINET_TENSOR) {
                LOGTBLEVEL1( m_pSimulationLog,"\t buildMode: BINET" << std::endl);
            } else if(m_settings.m_buildMode == GridBuilderSettings::BuildMode::MVBB) {
                LOGTBLEVEL1( m_pSimulationLog,"\t buildMode: MVBB" << std::endl);
            } else {
                GRSF_ERRORMSG("No BUILD implemented!")
            }


            LOGTBLEVEL1( m_pSimulationLog,
                    "\t aligned: "<< m_aligned << std::endl <<
                    "\t min/max: "<<
                    " [ " << m_aabb.m_minPoint.transpose() << " , " << m_aabb.m_maxPoint.transpose() << "]" << std::endl <<
                    "\t A_IK: \n" << m_A_IK << std::endl <<
                    "\t dim: " << "[ " << m_processDim.transpose() << "]" << std::endl <<
                    "\t extent: " << "[ " << m_aabb.extent().transpose() << "]" << std::endl;);
            m_pProcCommunicator->createProcTopoGrid(m_aabb,
                                                    m_processDim,
                                                    m_aligned,
                                                    m_A_IK);

            LOGTBLEVEL1( m_pSimulationLog,"---> GridTopoBuilder: Initialize ProcessCommunicator buffers..." << std::endl);
            m_pProcCommunicator->initializeNeighbourBuffers();

        }

        /** send the grid: only master rank executes this */
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
            std::generate_n(std::back_inserter(ranks), this->m_pProcCommunicator->getNProcesses()-1, [&]() {
                if(count ==masterRank) {
                    return (++count)++;
                } else {
                    return count++;
                }
            });


            for(auto rank : ranks) {
                LOGTBLEVEL1( m_pSimulationLog, "---> GridTopoBuilder: Sending grid to rank: "<< rank <<std::endl; );
                m_messageWrapperResults.setRank(rank);
                m_pProcCommunicator->sendMessageToRank(m_messageWrapperResults,rank, m_messageWrapperResults.m_tag);
            }
            // ===========================================================


            // Safty check:
            GRSF_ASSERTMSG(m_bodiesPerRank.size() == 0,"All states should be sent!");
        }


        /** receive grid results from master: all other ranks execute this */
        void receiveGrid(RankIdType masterRank){

            // All initial states get saved in m_pDynSys->m_bodiesInitStates
            m_messageWrapperResults.resetBeforLoad();
            m_pProcCommunicator->receiveMessageFromRank( m_messageWrapperResults,
                    masterRank,
                    m_messageWrapperResults.m_tag);

        }

        // master sorts bodies
        void sortBodies(RankIdType masterRank ) {
            LOGTBLEVEL3( m_pSimulationLog, "---> GridTopoBuilder: Sort bodies " << std::endl;);

            // Sort states acoording to their rank their in!
            m_bodiesPerRank.clear();
            m_pDynSys->m_bodiesInitStates.clear();

            RankIdType ownerRank;
            for(auto & p : m_initStates) {

                m_pProcCommunicator->getProcTopo()->belongsPointToProcess(p.second.getPosition(),ownerRank);

                if(ownerRank == masterRank) {
                    // move own body states in dynamic system init states
                    m_pDynSys->m_bodiesInitStates.insert(std::make_pair(p.second.m_id,p.second));
                } else {
                    // move other body states to a list for sending
                    m_bodiesPerRank[ownerRank].push_back(&p.second);
                }

            }

            // Log stuff
            std::stringstream ss;
            for(auto & s : m_bodiesPerRank) {
                LOGTBLEVEL3(m_pSimulationLog, "\t Bodies for rank " << s.first << " : ");
                for(auto & state : s.second) {
                    LOGTBLEVEL3(m_pSimulationLog,  RigidBodyId::getBodyIdString(state->m_id) << " , ");
                }
                LOGTBLEVEL3(m_pSimulationLog, std::endl);
            }
        }


        void writeTopo(PREC buildTime){
            #ifdef TOPOLOGY_BUILDER_WRITE_TOPO
                #ifdef TOPOLOGY_BUILDER_WRITE_PREDICTED_POINTS
                    const bool writePoints = true;
                #else
                    const bool writePoints = false;
                #endif

                #ifdef TOPOLOGY_BUILDER_WRITE_NEAREST_DISTANCES
                    const bool writeHistogramm = true;
                #else
                    const bool writeHistogramm = false;
                #endif
                writeTopoInfo<true,writePoints,writeHistogramm>(m_currentTime,buildTime);
            #endif
        }

        template<bool writeAABBList = true,
                 bool writePoints = false,
                 bool writeHistogram = false>
        void writeTopoInfo(PREC currentTime, PREC buildTime) {
            boost::filesystem::path filePath = m_localSimFolderPath;

            std::string filename = "TopologyInfo_" + std::to_string(m_builtTopologies);
            filePath /= filename + ".xml";

            LOGTBLEVEL3(m_pSimulationLog, "Write Grid Data to: " << filePath <<std::endl;);

            // Open XML and write structure!
            pugi::xml_document dataXML;
            std::stringstream xml("<TopologyBuilder type=\"Grid\" buildMode=\"\" >"
                    "<Description>"
                        "A_IK is tranformation matrix, which transforms points from frame K to frame I\n"
                        "AABB is in K Frame\n"
                        "AABBList contains all AABBs from all ranks in frame I\n"
                        "Time is the current simulation time\n"
                        "BuiltTime is the elapsed time to build the topology\n"
                    "</Description>"
                    "<Grid aligned=\"\" >"
                        "<Dimension/>"
                        "<MinPoint/>"
                        "<MaxPoint/>"
                        "<A_IK/>"
                    "</Grid>"
                    "</TopologyBuilder>");

            bool r = dataXML.load(xml);
            GRSF_ASSERTMSG(r,"Could not load initial xml data file");

            // Write data

            using XMLNodeType = pugi::xml_node;
            XMLNodeType node;
            static const auto  nodePCData = pugi::node_pcdata;
            XMLNodeType root =  dataXML.child("TopologyBuilder");

            std::string buildMode ="nothing";
            if(  m_settings.m_buildMode == GridBuilderSettings::BuildMode::ALIGNED) {
                buildMode="Aligned";
            } else if(m_settings.m_buildMode == GridBuilderSettings::BuildMode::PREDEFINED) {
                buildMode="Predefined";
            } else if (m_settings.m_buildMode == GridBuilderSettings::BuildMode::BINET_TENSOR) {
                buildMode="BinetTensor";
            } else if(m_settings.m_buildMode == GridBuilderSettings::BuildMode::MVBB) {
                buildMode="MVBB";
            }
            root.attribute("buildMode").set_value(buildMode.c_str()) ;

            // Write standart stuff
            Base::template writeTopoInfo<writeAABBList,writePoints,writeHistogram>(root,currentTime,buildTime);

            // Write Grid
            auto gridN = root.first_element_by_path("./Grid");

            gridN.attribute("aligned").set_value( m_aligned);

            node = gridN.first_element_by_path("./MinPoint");
            node.append_child(nodePCData).set_value( Utilities::typeToString(m_aabb.m_minPoint.transpose().format(MyMatrixIOFormat::SpaceSep)).c_str() );

            node = gridN.first_element_by_path("./MaxPoint");
            node.append_child(nodePCData).set_value( Utilities::typeToString(m_aabb.m_maxPoint.transpose().format(MyMatrixIOFormat::SpaceSep)).c_str() );

            node = gridN.first_element_by_path("./A_IK");
            node.append_child(nodePCData).set_value( Utilities::typeToString(m_A_IK.format(MyMatrixIOFormat::SpaceSep)).c_str() );

            node = gridN.first_element_by_path("./Dimension");
            node.append_child(nodePCData).set_value( Utilities::typeToString(m_processDim.transpose().format(MyMatrixIOFormat::SpaceSep)).c_str() );

            dataXML.save_file(filePath.c_str(),"    ");
        }

    };

    template<typename TProcCommunicator>
    class KdTreeTopologyBuilder : public GridTopologyBuilderBase<TProcCommunicator, KdTreeBuilderSettings> {
    public:
        using Base = GridTopologyBuilderBase<TProcCommunicator, KdTreeBuilderSettings>;
        DEFINE_TOPOLOGYBUILDERGRID_BASE_TYPES(Base)

    public:

        using TreeType       = KdTree::Tree< KdTree::TreeTraits<> >;              ///< Standart kdTree is already setup to use std::vector<Vector3 *>
        using TreeSimpleType = KdTree::TreeSimpleS<>; ///< Standart serializable simple kdTree is already setup to use std::vector<Vector3 *>

    private:

        friend class ParserModulesCreatorTopoBuilder<KdTreeTopologyBuilder>;
        friend class TopologyBuilderMessageWrapperResults<KdTreeTopologyBuilder>;


    private:

        unsigned int m_nGlobalSimBodies = 0; ///< Only for a check with MassPoints

        /** GLOBAL STUFF ================================================================ */
        unsigned int m_processes;
        std::unique_ptr<TreeType> m_kdTree_temp;
        std::shared_ptr<TreeSimpleType> m_kdTree_glo;

        using LeafNeighbourMapType =  typename TreeType::LeafNeighbourMapType;
        LeafNeighbourMapType m_neighbours;
        /** ============================================================================= */

        TopologyBuilderMessageWrapperResults<KdTreeTopologyBuilder> m_messageWrapperResults;



    public:

        KdTreeTopologyBuilder(std::shared_ptr<DynamicsSystemType> pDynSys,
                                std::shared_ptr<ProcessCommunicatorType > pProcCommunicator,
                                const TopologyBuilderSettings::RebuildSettings & rebuildSettings,
                                const TopologyBuilderSettings::MassPointPredSettings & massPointPredSettings,
                                const OutlierFilterSettingsType & globalOutlierFilterSettings,
                                const SettingsType & settings,
                                boost::filesystem::path sceneFilePath,
                                unsigned int nGlobalSimBodies)
            :Base(settings,massPointPredSettings, globalOutlierFilterSettings, sceneFilePath,
            TopologyBuilderEnumType::KDTREEBUILDER, pDynSys, pProcCommunicator,rebuildSettings),
             m_nGlobalSimBodies(nGlobalSimBodies),
             m_messageWrapperResults(this)
        {
        }



        void initTopology(PREC currentTime = 0) {
            LOGTB(m_pSimulationLog,"---> KdTreeTopoBuilder: initialize Topology" <<std::endl;)

            START_TIMER(startTopoTime)

            // get initial startTime which was parsed in ( gets overwritten by global initial condition if specified to continue from)
            m_timeStepperSettings.m_startTime = m_pDynSys->getSettingsTimeStepper().m_startTime;

            // increment number of built topologies
            m_currentTime = currentTime;
            ++m_builtTopologies;

            RankIdType masterRank = this->m_pProcCommunicator->getMasterRank();
            if(this->m_pProcCommunicator->hasMasterRank()) {


                // Parse all initial condition from the scene file (global initial condition is read too)
                ParserModulesCreatorTopoBuilder<KdTreeTopologyBuilder> c(this);
                SceneParserMPI<KdTreeTopologyBuilder, ParserModulesCreatorTopoBuilder<KdTreeTopologyBuilder>::template SceneParserTraits> parser(c, m_pSimulationLog,
                        ApplicationCLOptions::getSingleton().getMediaDir()) ; // this class is the modules generator

                // clean init states:
                m_initStates.clear();
                parser.parseScene(m_sceneFilePath);

                if(m_initStates.size() != m_nGlobalSimBodies) {
                    GRSF_ERRORMSG("Parsed to little initial states in scene file: " << m_sceneFilePath << " states: " << m_initStates.size() << "globalSimBodies: " << m_nGlobalSimBodies <<std::endl);
                }

                LOGTBLEVEL2(m_pSimulationLog, "---> KdTreeTopoBuilder: parsed states: "<<std::endl;);
                m_massPoints.clear();
                m_massPoints.reserve(m_initStates.size());
                for(auto & s : m_initStates) {
                    // Save mass points
                    m_massPoints.emplace_back( &s.second );
                    LOGTBLEVEL3(m_pSimulationLog, "\t\t state @"<<&s.second <<" : "
                            << RigidBodyId::getBodyIdString(s.second.m_id )<< " , " << s.second.m_q.transpose() << std::endl;);

                }

                this->initMakeBoundingBoxGlobal(masterRank);

                buildKdTree();

                buildTopo();
                sendKdTree(masterRank);

                STOP_TIMER_SEC(buildTime, startTopoTime)

                writeTopo(buildTime);


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

            LOGTBLEVEL1(m_pSimulationLog, "---> KdTreeTopoBuilder: States in m_bodiesInitStates: (" << m_pDynSys->m_bodiesInitStates.size()<< ")" <<std::endl;);
            for(auto & s :  m_pDynSys->m_bodiesInitStates) {
                LOGTBLEVEL3(m_pSimulationLog, "\t\t state: "
                        << RigidBodyId::getBodyIdString(s.second.m_id )<< " , q: "
                        << s.second.m_q.transpose() << " u: " << s.second.m_u.transpose() << std::endl;);
            }



            // Write m_startTime to TimeStepperSettings in DynamicsSystem (if it stayed the same , it does not matter)
            m_pDynSys->setStartTime( m_timeStepperSettings.m_startTime);


            LOGTB(m_pSimulationLog,"---> KdTreeTopoBuilder: init topology finished!" <<std::endl;)
        }

        void rebuildTopology(PREC currentTime) {

            LOGTB(m_pSimulationLog,"---> KdTreeTopoBuilder: rebuild Topology" <<std::endl;)

            START_TIMER(startTopoTime)


            // increment number of built topologies
            ++m_builtTopologies;
            m_currentTime = currentTime;



            LOGTB(m_pSimulationLog,"---> KdTreeTopoBuilder: Clear all init states in DynamicsSystem" <<std::endl;)
            m_pDynSys->m_bodiesInitStates.clear();

            RankIdType masterRank = this->m_pProcCommunicator->getMasterRank();
            if(this->m_pProcCommunicator->hasMasterRank()) {

                // compute local stuff for the corresponding build method
                this->template rebuildMakeBoundingBoxLocal_PreSend<true>(masterRank);

                // Receive Messages
                // Fill linear array with all ranks except master
                std::vector<RankIdType> ranks;
                unsigned int count=0;
                std::generate_n(std::back_inserter(ranks),
                        this->m_pProcCommunicator->getNProcesses()-1,
                [&]() {
                    if(count == masterRank) {
                        return (++count)++;
                    } else {
                        return count++;
                    }
                });

                m_messageBodies.resetBeforLoad(); // Resets all variables for the appropriate build method in this class
                this->m_pProcCommunicator->receiveMessageFromRanks(m_messageBodies, ranks, m_messageBodies.m_tag);

                //Check if number of masspoints received is equal to the total in the simulation
                if(m_nGlobalSimBodies != m_initStates.size() ) {
                    GRSF_ERRORMSG("m_nGlobalSimBodies: " << m_nGlobalSimBodies << "not equal to" <<  m_initStates.size() <<std::endl);
                }


                this->rebuildMakeBoundingBoxGlobal(masterRank);

                buildKdTree();

                buildTopo();
                sendKdTree(masterRank);

                STOP_TIMER_SEC(buildTime, startTopoTime)

                // Write grid data to file
                writeTopo(buildTime);

                cleanUpGlobal();
                cleanUpLocal();

            } else {

                // compute local stuff for the corresponding build method
                this->template rebuildMakeBoundingBoxLocal_PreSend<false>(masterRank);

                this->m_pProcCommunicator->sendMessageToRank(m_messageBodies, masterRank, m_messageBodies.m_tag);

                this->rebuildMakeBoundingBoxLocal_PostSend(masterRank);

                receiveKdTree(masterRank);

                buildTopo();
                cleanUpLocal();
            }

            LOGTBLEVEL1(m_pSimulationLog, "---> KdTreeTopoBuilder: States in m_bodiesInitStates: (" << m_pDynSys->m_bodiesInitStates.size()<< ")" <<std::endl;);
            for(auto & s :  m_pDynSys->m_bodiesInitStates) {
                LOGTBLEVEL3(m_pSimulationLog, "\t\t state: "
                        << RigidBodyId::getBodyIdString(s.second.m_id )<< " , q: "
                        << s.second.m_q.transpose() << " u: " << s.second.m_u.transpose() << std::endl;);
            }

            LOGTB(m_pSimulationLog,"---> KdTreeTopoBuilder: rebuild topology finished!" <<std::endl;)

        }

    private:

        void cleanUpGlobal() {
            Base::cleanUpGlobal();

            m_kdTree_temp.reset();
            m_kdTree_glo.reset();
        }

        void cleanUpLocal() {
            Base::cleanUpLocal();
        }

        // DEBUG
        template<typename Container>
        void dumpPoints(std::string filePath, Container & c) {

            std::ofstream l;
            l.open(filePath.c_str());
            if(!l.good()){
                ApproxMVBB_ERRORMSG("Could not open file: " << filePath << std::endl)
            }

            for(auto & v: c) {
                l << v.transpose().format(MyMatrixIOFormat::SpaceSep) << std::endl;
            }
            l.close();
        }

        template<typename Container>
        void dumpPointsBinary(std::string filePath, const Container & v) {

            std::ofstream l;
            l.open(filePath.c_str(),std::ios::binary);
            if(!l.good()){
                ApproxMVBB_ERRORMSG("Could not open binary file: " << filePath << std::endl)
            }
            for(auto & vec : v){
                l.write(reinterpret_cast<const char*>(vec.data()), vec.size()*sizeof(PREC));
            }
            l.close();
        }
        // DEBUG


        void buildKdTree() {

            using SplitHeuristicType = TreeType::SplitHeuristicType;
            using NodeType = TreeType::NodeType;
            using NodeDataType = TreeType::NodeDataType;
            using BoundaryInfoType = typename NodeType::BoundaryInfoType;
            // static const unsigned int Dimension = NodeDataType::Dimension;
            using PointListType = NodeDataType::PointListType;

            // transform the points if not aligned into the aabbs frame
            if(!this->m_aligned){
                for(auto & v : m_predPoints){
                    v = this->m_A_IK.transpose()*v;
                }
            }


            // make pointer list
            auto s = m_predPoints.size();
            PointListType * vec ( new PointListType(s) ); // will be managed by unique pointer later
            for(std::size_t i= 0; i<s; ++i) {
                (*vec)[i] = &m_predPoints[i];
            }


            typename SplitHeuristicType::QualityEvaluator e(0.0, /* splitratio (maximized by MidPoint) */
                    0.0, /* pointratio (maximized by MEDIAN)*/
                    1.0);/* extentratio (maximized by MidPoint)*/

            m_kdTree_temp.reset(new TreeType());

            m_kdTree_temp->initSplitHeuristic( std::initializer_list<SplitHeuristicType::Method>({
                                            SplitHeuristicType::Method::MEDIAN
                                            /*SplitHeuristicType::Method::GEOMETRIC_MEAN,
                                              SplitHeuristicType::Method::MIDPOINT*/
                                        }),
                                        m_settings.m_minPointsForSplit, m_settings.m_minCellSize,
                                        SplitHeuristicType::SearchCriteria::FIND_BEST,
                                        e,
                                        0.0, 0.0, 0.0);

            auto rootData = std::unique_ptr<NodeDataType>(new NodeDataType(vec->begin(),vec->end(),
                    std::unique_ptr<PointListType>(vec) ));

            LOGTBLEVEL1(m_pSimulationLog, "---> KdTreeTopoBuilder: Building KdTree with : "
                        << "minP: " << this->m_aabb.m_minPoint.transpose() << " maxP: " << this->m_aabb.m_maxPoint.transpose() << std::endl
                        << "points: " << vec->size() << std::endl
                        )

            m_kdTree_temp->build(m_aabb,std::move(rootData), m_settings.m_maxTreeDepth, m_pProcCommunicator->getNProcesses());

            // build the neighbours according to statistics
            m_neighbours = m_kdTree_temp->buildLeafNeighboursAutomatic();

            LOGTBLEVEL1(m_pSimulationLog,m_kdTree_temp->getStatisticsString())

            m_processes = m_kdTree_temp->getLeafs().size();

            // TODO
            // If we found less processes which need to continue the simulation,
            // all other remaining ones go into sleep state! -> implement this in simulation manager
            if(m_processes != m_pProcCommunicator->getNProcesses()) {
                GRSF_ERRORMSG("KdTree could only determine " << m_processes << " processes but " << m_pProcCommunicator->getNProcesses() << " are needed!")
            }

            // copy global kdTree to a simple version
            m_kdTree_glo.reset(new TreeSimpleType{*m_kdTree_temp});

            // make simple version filling the whole space for sharing
            for(auto * n : m_kdTree_glo->getNodes() ){

                auto & bounds = n->getBoundaries();
                for(std::size_t i=0;i< TreeType::Dimension; ++i){

                    if( bounds.at(i,0) == nullptr ){
                        // boundary at minimum -> extent to max
                        n->aabb().expandToMaxExtent<false>(i);
                    }

                    if( bounds.at(i,1) == nullptr ){
                        // boundary at maximum -> extent to lowest
                        n->aabb().expandToMaxExtent<true>(i);
                    }

                }
            }
        }

        void buildTopo() {

            LOGTBLEVEL1(m_pSimulationLog, "---> KdTreeTopoBuilder: build grid ..." << std::endl;);

            if(m_rebuildSettings.m_mode == RebuildSettingsType::Mode::STATIC) {
                LOGTBLEVEL1( m_pSimulationLog, "---> KdTreeTopoBuilder: Create static ProcessTopologyKdTree : " << std::endl;)
            } else {
                LOGTBLEVEL1( m_pSimulationLog, "---> KdTreeTopoBuilder: Create new ProcessTopologyKdTree: "<< std::endl;)
            }

            if(  m_settings.m_buildMode == KdTreeBuilderSettings::BuildMode::ALIGNED) {
                LOGTBLEVEL1( m_pSimulationLog,"\t buildMode: ALIGNED" << std::endl);
            } else if(m_settings.m_buildMode == KdTreeBuilderSettings::BuildMode::PREDEFINED) {
                LOGTBLEVEL1( m_pSimulationLog, "\t buildMode: PREDEFINED" << std::endl );
            }
            else if (m_settings.m_buildMode == KdTreeBuilderSettings::BuildMode::BINET_TENSOR){
                 LOGTBLEVEL1( m_pSimulationLog,"\t buildMode: BINET" << std::endl);
            }
            else if(m_settings.m_buildMode == KdTreeBuilderSettings::BuildMode::MVBB) {
                LOGTBLEVEL1( m_pSimulationLog,"\t buildMode: MVBB" << std::endl);
            } else {
                GRSF_ERRORMSG("No BUILD implemented!")
            }

            for( auto & nbs : m_neighbours){
                LOGTBLEVEL3(m_pSimulationLog,"Neighbours for rank: " << nbs.first << ": ");
                for(auto & r : nbs.second){
                   LOGTBLEVEL3(m_pSimulationLog, r );
                }
                LOGTBLEVEL3(m_pSimulationLog,std::endl);
            }

            LOGTBLEVEL1( m_pSimulationLog,
                    "\t aligned: "<< m_aligned << std::endl <<
                    "\t min/max: "<<
                    " [ " << m_aabb.m_minPoint.transpose() << " , " << m_aabb.m_maxPoint.transpose() << "]" << std::endl <<
                    "\t A_IK: \n" << m_A_IK << std::endl <<
                    "\t processes: " << m_processes << std::endl <<
                    "\t extent: " << "[ " << m_aabb.extent().transpose() << "]" << std::endl;);
            m_pProcCommunicator->createProcTopoKdTree(  m_kdTree_glo,
                                                        m_neighbours,
                                                        m_aabb,
                                                        m_aligned,
                                                        m_A_IK);

            LOGTBLEVEL1( m_pSimulationLog,"---> KdTreeTopoBuilder: Initialize ProcessCommunicator buffers..." << std::endl);
            m_pProcCommunicator->initializeNeighbourBuffers();

        }

        //only master rank executes this
        void sendKdTree(RankIdType masterRank) {


            sortBodies(masterRank);

            // Send all data to the neighbours ===========================
            // For each rank r send:
            // - topology identifier (GRID)
            // - topology settings (GRID: min,max,dimension)
            // - send all body ids and initial states for the bodies in rank r


            // Fill linear array with all ranks except master
            std::vector<RankIdType> ranks;
            unsigned int count=0;
            std::generate_n(std::back_inserter(ranks), this->m_pProcCommunicator->getNProcesses()-1, [&]() {
                if(count ==masterRank) {
                    return (++count)++;
                } else {
                    return count++;
                }
            });

            for(auto rank : ranks){
                LOGTBLEVEL1( m_pSimulationLog, "---> KdTreeTopoBuilder: Sending grid to rank: "<< rank <<std::endl; );
                m_messageWrapperResults.setRank(rank);
                m_pProcCommunicator->sendMessageToRank(m_messageWrapperResults,rank, m_messageWrapperResults.m_tag);
            }
            // ===========================================================


            // Safty check:
            GRSF_ASSERTMSG(m_bodiesPerRank.size() == 0,"All states should be sent!");
        }

        void receiveKdTree(RankIdType masterRank){
            // Receive kdTree results from master;
            // All initial states get saved in m_pDynSys->m_bodiesInitStates
            m_messageWrapperResults.resetBeforLoad();
            m_pProcCommunicator->receiveMessageFromRank( m_messageWrapperResults,
                                                         masterRank,
                                                         m_messageWrapperResults.m_tag);
        }

        // master sorts bodies
        void sortBodies(RankIdType masterRank ) {
            LOGTBLEVEL3( m_pSimulationLog, "---> KdTreeTopoBuilder: Sort bodies " << std::endl;);

            // Sort states acoording to their rank their in!
            m_bodiesPerRank.clear();
            m_pDynSys->m_bodiesInitStates.clear();

            RankIdType ownerRank;
            for(auto & p : m_initStates) {

                m_pProcCommunicator->getProcTopo()->belongsPointToProcess(p.second.getPosition(),ownerRank);

                if(ownerRank == masterRank) {
                    // move own body states in dynamic system init states
                    m_pDynSys->m_bodiesInitStates.insert(std::make_pair(p.second.m_id,p.second));
                } else {
                    // move other body states to a list for sending
                    m_bodiesPerRank[ownerRank].push_back(&p.second);
                }
            }

            // Log stuff
            std::stringstream ss;
            for(auto & s : m_bodiesPerRank) {
                LOGTBLEVEL3(m_pSimulationLog, "\t Bodies for rank " << s.first << " : ");
                for(auto & state : s.second) {
                    LOGTBLEVEL3(m_pSimulationLog,  RigidBodyId::getBodyIdString(state->m_id) << " , ");
                }
                LOGTBLEVEL3(m_pSimulationLog, std::endl);
            }
        }


        void writeTopo(PREC buildTime){
            #ifdef TOPOLOGY_BUILDER_WRITE_TOPO
                #ifdef TOPOLOGY_BUILDER_WRITE_PREDICTED_POINTS
                    const bool writePoints = true;
                #else
                    const bool writePoints = false;
                #endif

                #ifdef TOPOLOGY_BUILDER_WRITE_NEAREST_DISTANCES
                    const bool writeHistogramm = true;
                #else
                    const bool writeHistogramm = false;
                #endif
                writeTopoInfo<true,writePoints,writeHistogramm>(m_currentTime,buildTime);
            #endif
        }

        template<bool writeAABBList = true,
                 bool writePoints = false,
                 bool writeHistogram = false>
        void writeTopoInfo(PREC currentTime, PREC buildTime) {
            boost::filesystem::path filePath = m_localSimFolderPath;

            std::string filename = "TopologyInfo_" + std::to_string(m_builtTopologies);
            filePath /= filename + ".xml";

            LOGTBLEVEL3(m_pSimulationLog, "Write Grid Data to: " << filePath <<std::endl;);

            // Open XML and write structure!
            pugi::xml_document dataXML;
            std::stringstream xml("<TopologyBuilder type=\"KdTree\" buildMode=\"\" >"
                    "<Description>"
                        "A_IK is tranformation matrix, which transforms points from frame K to frame I\n"
                        "AABBList contains all AABBs from all ranks in frame I\n"
                        "Time is the current simulation time\n"
                        "BuiltTime is the elapsed time to build the topology\n"
                        "AABBTree contains the tree without the leafs\n"
                    "</Description>"
                    "</TopologyBuilder>");

            bool r = dataXML.load(xml);
            GRSF_ASSERTMSG(r,"Could not load initial xml data file");

            // Write data

            using XMLNodeType = pugi::xml_node;
            XMLNodeType node;
            //static const auto  nodePCData = pugi::node_pcdata;
            XMLNodeType root =  dataXML.child("TopologyBuilder");


            std::string buildMode ="nothing";

            if(  m_settings.m_buildMode == KdTreeBuilderSettings::BuildMode::ALIGNED) {
                buildMode="Aligned";
            } else if(m_settings.m_buildMode == KdTreeBuilderSettings::BuildMode::PREDEFINED) {
                buildMode="Predefined";
            } else if (m_settings.m_buildMode == KdTreeBuilderSettings::BuildMode::BINET_TENSOR) {
                buildMode="BinetTensor";
            } else if(m_settings.m_buildMode == KdTreeBuilderSettings::BuildMode::MVBB) {
                buildMode="MVBB";
            }
            root.attribute("buildMode").set_value(buildMode.c_str()) ;

            // Write standart stuff
            Base::template writeTopoInfo<writeAABBList,writePoints,writeHistogram>(root,currentTime,buildTime);

            // Write kdTree
            m_kdTree_temp->appendToXML(root,m_aligned,m_A_IK);

            dataXML.save_file(filePath.c_str(),"    ");
        }

    };



}; //MPILayer


#endif

