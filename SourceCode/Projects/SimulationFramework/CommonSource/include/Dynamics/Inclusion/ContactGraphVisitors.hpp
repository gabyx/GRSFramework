#ifndef ContactGraphVisitors_hpp
#define ContactGraphVisitors_hpp

#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include InclusionSolverSettings_INCLUDE_FILE

#include "VectorToSkewMatrix.hpp"
#include "ProxFunctions.hpp"

/**
@brief Visitor for class ContactGraph<TRigidBody,ContactGraphMode::ForIteration>
*/
template<typename TContactGraph>
class SorProxStepNodeVisitor{
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

    using ContactGraphType = TContactGraph;
    using NodeDataType = typename ContactGraphType::NodeDataType;
    using EdgeDataType = typename ContactGraphType::EdgeDataType;
    using EdgeType = typename ContactGraphType::EdgeType;
    using NodeType = typename ContactGraphType::NodeType;


    SorProxStepNodeVisitor(const InclusionSolverSettingsType &settings,
                           bool & globalConverged,
                           const unsigned int & globalIterationNeeded,
                           ContactGraphType * graph):
        m_settings(settings),m_bConverged(globalConverged),
        m_globalIterationCounter(globalIterationNeeded),
        m_pGraph(graph)
    {}


    inline void visitNode(NodeType & node) {
        m_delegate(node);
    }

    void setLog(Logging::Log * solverLog) {
        m_pSolverLog = solverLog;
    }
    // Set Sor Prox parameter, before calling visitNode
    void setParams(PREC alpha) {
        m_alpha = alpha;
    }

//    template<int B>
//    void doVelocityUpdate(typename ContactGraphType::NodeDataType & nodeData) {
//        typedef decltype(nodeData.m_u1BufferPtr->m_front) VectorUType;
//        RigidBodyType * pBody;
//        VectorUType * pUBuffer;
//
//        if(B==1) {
//            pUBuffer = &nodeData.m_u1BufferPtr->m_front;
//            pBody = nodeData.m_pCollData->m_pBody1;
//        } else {
//            pUBuffer = &nodeData.m_u2BufferPtr->m_front;
//            pBody = nodeData.m_pCollData->m_pBody2;
//        }
//
//        // u_S + Minv *h * deltaT
//        *pUBuffer = pBody->m_pSolverData->m_uBegin + pBody->m_MassMatrixInv_diag.asDiagonal() * pBody->m_h_term * m_settings.m_deltaT;
//
//        // Iterate over all nodes and add contribution
//        auto nodeList = m_pGraph->m_simBodiesToContactsMap[pBody];
//        for(auto it = nodeList.begin(); it!=nodeList.end(); ++it) {
//            *pUBuffer += pBody->m_MassMatrixInv_diag.asDiagonal() * ContactGraphType::getW_bodyRef((*it)->m_nodeData,pBody) * (*it)->m_nodeData.m_LambdaFront;
//        }
//
//    }

    void setNewDamping(typename ContactGraphType::NodeDataType & nodeData) {
        using CMT = typename CONTACTMODELTYPE(ContactModels::Enum::UCFD);
        nodeData.m_contactParameter.m_params[CMT::d_NIdx] = 1e-6;
        nodeData.m_contactParameter.m_params[CMT::d_TIdx] = 1e-6;
        recalculateR(nodeData, nodeData.m_contactParameter);
    }

    void recalculateR(typename ContactGraphType::NodeDataType & nodeData, ContactParameter & contactParameter) {

        nodeData.m_G_ii.setZero();
        if(nodeData.m_pCollData->m_pBody1->m_eMode == RigidBodyType::BodyMode::SIMULATED) {
            nodeData.m_G_ii += nodeData.m_W_body1.transpose() * nodeData.m_pCollData->m_pBody1->m_MassMatrixInv_diag.asDiagonal() * nodeData.m_W_body1 ;
        }
        // SECOND BODY!
        if(nodeData.m_pCollData->m_pBody2->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {
            nodeData.m_G_ii += nodeData.m_W_body2.transpose() * nodeData.m_pCollData->m_pBody2->m_MassMatrixInv_diag.asDiagonal() * nodeData.m_W_body2 ;
        }




        if(nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCFD) {
            using CMT = typename CONTACTMODELTYPE(ContactModels::Enum::UCFD);
            Vector3 dinv(contactParameter.m_params[CMT::d_NIdx], //d_N
                         contactParameter.m_params[CMT::d_TIdx], //d_T
                         contactParameter.m_params[CMT::d_TIdx]); //d_T
            nodeData.m_G_ii.diagonal() += 1.0/m_settings.m_deltaT*dinv;
        } else if(nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCFDD) {
            using CMT = typename CONTACTMODELTYPE(ContactModels::Enum::UCFDD);
            Vector3 dinv;
            dinv(0) = contactParameter.m_params[CMT::d_NIdx];
            // if lambda_N <= eps, set damping to d_Tfix
            if( std::abs(nodeData.m_LambdaBack(0)) <= contactParameter.m_params[CMT::epsIdx] ) {
                dinv.tail<2>().setConstant( contactParameter.m_params[CMT::d_TfixIdx] );
            } else { //dinvT = gammaMax / (mu *lambdaN)
                dinv.tail<2>().setConstant( contactParameter.m_params[CMT::gamma_maxIdx] / (contactParameter.m_params[CMT::muIdx]*nodeData.m_LambdaBack(0) ) );
            }
            nodeData.m_G_ii.diagonal() += 1.0/m_settings.m_deltaT*dinv;
        }

        // Calculate R_ii
        // Take also offdiagonal values for lambda_N
        //nodeData.m_R_i_inv_diag(0) = m_alpha / std::max(std::max(nodeData.m_G_ii(0,0), nodeData.m_mu(0)*nodeData.m_G_ii(0,1)), nodeData.m_mu(0)*nodeData.m_G_ii(0,2));
        // Take only diagonal
        nodeData.m_R_i_inv_diag(0) = m_alpha / nodeData.m_G_ii(0,0);
        PREC r_T = m_alpha / ((nodeData.m_G_ii.diagonal().template tail<2>()).maxCoeff());
        nodeData.m_R_i_inv_diag(1) = r_T;
        nodeData.m_R_i_inv_diag(2) = r_T;
    }


    // Function delegate to make class visitable
    typedef srutil::delegate1<void,NodeType &> VisitNodeDelegate;
protected:
    Logging::Log * m_pSolverLog = nullptr;
    PREC m_alpha;
    const InclusionSolverSettingsType & m_settings;
    bool & m_bConverged; ///< Access to global flag for cancelation criteria
    const unsigned int & m_globalIterationCounter; ///< Access to global iteration counter

    ContactGraphType * m_pGraph = nullptr;


    VisitNodeDelegate m_delegate;

};

#define DEFINE_SORPROXSTEPNODE_BASE \
    using Base = SorProxStepNodeVisitor<TContactGraph>; \
    using ContactGraphType = typename Base::ContactGraphType;\
    using NodeDataType = typename Base::NodeDataType;\
    using EdgeDataType = typename Base::EdgeDataType;\
    using EdgeType = typename Base::EdgeType;\
    using NodeType = typename Base::NodeType;\
    using VisitNodeDelegate = typename Base::VisitNodeDelegate;\
    using Base::m_pSolverLog;\
    using Base::m_settings;\
    using Base::m_pGraph;\
    using Base::m_delegate;\
    using Base::m_globalIterationCounter;\
    using Base::m_alpha;\
    using Base::m_bConverged;

/**
* This is a contact sor, projects one contact together!
*/
template<typename TContactGraph>
class ContactSorProxStepNodeVisitor : public SorProxStepNodeVisitor<TContactGraph> {
public:
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

    DEFINE_SORPROXSTEPNODE_BASE


    ContactSorProxStepNodeVisitor(const InclusionSolverSettingsType &settings,
                                  bool & globalConverged,
                                  const unsigned int & globalIterationNeeded,
                                  ContactGraphType * graph):
        Base(settings,globalConverged,globalIterationNeeded,graph)
    {
        this->m_delegate = VisitNodeDelegate::template from_method<
              ContactSorProxStepNodeVisitor , &ContactSorProxStepNodeVisitor::visitNode >(this);
    }

    void visitNode(NodeType& node) {
        /* Convergence Criterias are no more checked if the m_bConverged (gloablConverged) flag is already false
           Then the iteration is not converged somewhere, and we need to wait till the next iteration!
        */
        typename ContactGraphType::NodeDataType & nodeData = node.m_nodeData;
        static VectorUBody uCache1,uCache2;
        PREC residual;


        LOGSLLEVEL3_CONTACT(m_pSolverLog, "---> SorProx, Node: " << node.m_nodeNumber <<"====================="<<  std::endl);
        if( nodeData.m_pCollData->m_pBody1->m_eMode == RigidBodyType::BodyMode::SIMULATED  &&  nodeData.m_pCollData->m_pBody2->m_eMode == RigidBodyType::BodyMode::SIMULATED) {
            LOGSLLEVEL3_CONTACT(m_pSolverLog, "\t---> Sim<->Sim Node:"<<  std::endl);
        }


        if( nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCF ||
            nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCFD ||
            nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCFDD  ) {

            using CMT = typename CONTACTMODELTYPE(ContactModels::Enum::UCF);

            // Init the prox value
            nodeData.m_LambdaFront = nodeData.m_b;

            // FIRST BODY!
            if( nodeData.m_pCollData->m_pBody1->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {
                nodeData.m_LambdaFront += nodeData.m_W_body1.transpose() * nodeData.m_u1BufferPtr->m_front ;
            }
            // SECOND BODY!
            if( nodeData.m_pCollData->m_pBody2->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {
                nodeData.m_LambdaFront += nodeData.m_W_body2.transpose() * nodeData.m_u2BufferPtr->m_front;
            }

            // Experimental
            //Relaxation term damper (take care R_i_inv needs to be initialized as well!)
            if(nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCFD) {
                using CMT = typename CONTACTMODELTYPE(ContactModels::Enum::UCFD);
                //Drive damping to zero after some iterations:
                //if (m_globalIterationCounter == 300) {
                    //nodeData.m_contactParameter.m_params[4] = 1e-7;
                //}

                nodeData.m_LambdaFront(0) += nodeData.m_LambdaBack(0) * nodeData.m_contactParameter.m_params[CMT::d_NIdx] / m_settings.m_deltaT;
                nodeData.m_LambdaFront(1) += nodeData.m_LambdaBack(1) * nodeData.m_contactParameter.m_params[CMT::d_TIdx] / m_settings.m_deltaT;
                nodeData.m_LambdaFront(2) += nodeData.m_LambdaBack(2) * nodeData.m_contactParameter.m_params[CMT::d_TIdx] / m_settings.m_deltaT;


            } else if(nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCFDD) {
                using CMT = typename CONTACTMODELTYPE(ContactModels::Enum::UCFDD);
                this->recalculateR(nodeData,nodeData.m_contactParameter);
                // if lambda_N <= eps, set damping to d_Tfix
                PREC dinvT;
                if( std::abs(nodeData.m_LambdaBack(0)) <= nodeData.m_contactParameter.m_params[CMT::epsIdx] ) {
                    dinvT = nodeData.m_contactParameter.m_params[CMT::d_TfixIdx];
                } else { //dinvT = gammaMax / (mu *lambdaN)
                    dinvT = nodeData.m_contactParameter.m_params[CMT::gamma_maxIdx] / (nodeData.m_contactParameter.m_params[CMT::muIdx]*nodeData.m_LambdaBack(0) );
                }

                nodeData.m_LambdaFront(0) += nodeData.m_LambdaBack(0) * nodeData.m_contactParameter.m_params[CMT::d_NIdx] / m_settings.m_deltaT;
                nodeData.m_LambdaFront(1) += nodeData.m_LambdaBack(1) * dinvT / m_settings.m_deltaT;
                nodeData.m_LambdaFront(2) += nodeData.m_LambdaBack(2) * dinvT / m_settings.m_deltaT;
            }

            //Prox
            // HACK drive up parameter mu, does not help
//            PREC mu =  nodeData.m_mu(0);
//            if(m_globalIterationCounter <= 10){
//                 mu = ((m_globalIterationCounter) / 10 ) * nodeData.m_mu(0);
//                 m_bConverged = false;
//            }
//            else if (m_globalIterationCounter <= 300) {
//                mu = ((m_globalIterationCounter-200) / 500 ) * nodeData.m_mu(0);
//                m_bConverged = false;
//            }
//            else{
//                mu =  nodeData.m_mu(0);
//            }

            // PROX  prox(lambda - R_i_inv * gamma) ==================================================================================
            if(m_settings.m_eMethod == InclusionSolverSettingsType::SOR_CONTACT_DS){
                // De Saxe Formulation
                // add correction term mu*gammaT to gammaN (for DeSaxce Cone Formulation)
                nodeData.m_LambdaFront(0) += nodeData.m_contactParameter.m_params[CMT::muIdx]*nodeData.m_LambdaFront.template tail<2>().norm();
                //Multiply R_i_inverse
                nodeData.m_LambdaFront = -(nodeData.m_R_i_inv_diag.asDiagonal() * nodeData.m_LambdaFront).eval(); //No alias due to diagonal!!! (if normal matrix multiplication there is aliasing!
                nodeData.m_LambdaFront += nodeData.m_LambdaBack;
                //Prox onto friction cone
                Prox::ProxFunction<ConvexSets::Cone3D>::doProxSingle( nodeData.m_contactParameter.m_params[CMT::muIdx],nodeData.m_LambdaFront);
            }else{
                // Alart Curnier Formulation
                //Multiply R_i_inverse
                nodeData.m_LambdaFront = -(nodeData.m_R_i_inv_diag.asDiagonal() * nodeData.m_LambdaFront).eval(); //No alias due to diagonal!!! (if normal matrix multiplication there is aliasing!
                nodeData.m_LambdaFront += nodeData.m_LambdaBack;
                //Prox onto R+ and Disk
                Prox::ProxFunction<ConvexSets::RPlusAndDisk>::doProxSingle( nodeData.m_contactParameter.m_params[CMT::muIdx],nodeData.m_LambdaFront);
            }


            LOGSLLEVEL3_CONTACT(m_pSolverLog, "\t---> nd.m_LambdaBack: "  << nodeData.m_LambdaBack.transpose() << std::endl);
            LOGSLLEVEL3_CONTACT(m_pSolverLog, "\t---> nd.m_LambdaFront: " << nodeData.m_LambdaFront.transpose() << std::endl);
            if(Numerics::cancelCriteriaValue(nodeData.m_LambdaBack,nodeData.m_LambdaFront,m_settings.m_AbsTol, m_settings.m_RelTol)) {
                LOGSLLEVEL3_CONTACT(m_pSolverLog,"\t---> Lambda converged"<<std::endl);
            }

            // Velocity Updates ====================================================================================================================
            // u_k+1 = u_k + M^-1 W (lambda_k+1 - lambda_k)
            // FIRST BODY!
            decltype(nodeData.m_LambdaFront) deltaLambda = nodeData.m_LambdaFront - nodeData.m_LambdaBack ;
            if( nodeData.m_pCollData->m_pBody1->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {
                uCache1 = nodeData.m_u1BufferPtr->m_front;

                // Velocity update (wahrscheinlich Auslöschung bei Lambda)
                nodeData.m_u1BufferPtr->m_front = uCache1
                                                  + nodeData.m_pCollData->m_pBody1->m_MassMatrixInv_diag.asDiagonal() * nodeData.m_W_body1 * deltaLambda;

                //Sepcial update (no differences)
                //doVelocityUpdate<1>(nodeData);


                LOGSLLEVEL3_CONTACT(m_pSolverLog,"\t---> nd.u1Front: " << nodeData.m_u1BufferPtr->m_front.transpose() << std::endl);


                if(m_settings.m_eConvergenceMethod == InclusionSolverSettingsType::InVelocityLocal) {
                    if(m_globalIterationCounter >= m_settings.m_MinIter && m_bConverged) {
                        nodeData.m_bConverged  = Numerics::cancelCriteriaValue(uCache1,nodeData.m_u1BufferPtr->m_front,m_settings.m_AbsTol, m_settings.m_RelTol);
                        if(!nodeData.m_bConverged ) {
                            //converged stays false;
                            // Set global Converged = false;
                            m_bConverged = false;
                            *m_pSolverLog << "\t---> m_bConverged = false;"<<std::endl;
                        }
                    } else {
                        m_bConverged=false;
                    }
                } else if(m_settings.m_eConvergenceMethod == InclusionSolverSettingsType::InEnergyLocalMix) {
                    if(m_globalIterationCounter >= m_settings.m_MinIter && m_bConverged) {
                        nodeData.m_bConverged  = Numerics::cancelCriteriaMatrixNorm(      uCache1,
                                                 nodeData.m_pCollData->m_pBody1->m_MassMatrix_diag,
                                                 nodeData.m_LambdaBack,
                                                 nodeData.m_LambdaFront,
                                                 nodeData.m_G_ii,
                                                 m_settings.m_AbsTol,
                                                 m_settings.m_RelTol);
                        if(!nodeData.m_bConverged ) {
                            //converged stays false;
                            // Set global Converged = false;
                            m_bConverged = false;
                        }
                    } else {
                        m_bConverged=false;
                    }
                }

            }
            // SECOND BODY
            if( nodeData.m_pCollData->m_pBody2->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {
                uCache2 = nodeData.m_u2BufferPtr->m_front;

                // Velocity update (wahrscheinlich Auslöschung bei Lambda)
                nodeData.m_u2BufferPtr->m_front = uCache2
                                                  + nodeData.m_pCollData->m_pBody2->m_MassMatrixInv_diag.asDiagonal() * nodeData.m_W_body2 * deltaLambda ;

                //Sepcial update (no differences)
                //doVelocityUpdate<2>(nodeData);

                LOGSLLEVEL3_CONTACT(m_pSolverLog,"\t---> nd.u2Front: " << nodeData.m_u2BufferPtr->m_front.transpose() << std::endl);

                if(m_settings.m_eConvergenceMethod == InclusionSolverSettingsType::InVelocityLocal) {
                    if(m_globalIterationCounter >= m_settings.m_MinIter && m_bConverged) {
                        nodeData.m_bConverged  = Numerics::cancelCriteriaValue(uCache2,
                                                 nodeData.m_u2BufferPtr->m_front,
                                                 m_settings.m_AbsTol,
                                                 m_settings.m_RelTol);
                        if(!nodeData.m_bConverged ) {
                            //converged stays false;
                            // Set global Converged = false;
                            m_bConverged = false;
                        }
                    } else {
                        m_bConverged=false;
                    }

                } else if(m_settings.m_eConvergenceMethod == InclusionSolverSettingsType::InEnergyLocalMix) {
                    if(m_globalIterationCounter >= m_settings.m_MinIter && m_bConverged) {
                        nodeData.m_bConverged  = Numerics::cancelCriteriaMatrixNorm(   uCache2,
                                                 nodeData.m_pCollData->m_pBody2->m_MassMatrix_diag,
                                                 nodeData.m_LambdaBack,
                                                 nodeData.m_LambdaFront,
                                                 nodeData.m_G_ii,
                                                 m_settings.m_AbsTol,
                                                 m_settings.m_RelTol);
                        if(!nodeData.m_bConverged ) {
                            //converged stays false;
                            // Set global Converged = false;
                            m_bConverged = false;
                        }
                    } else {
                        m_bConverged=false;
                    }
                }
            }


            if(m_settings.m_eConvergenceMethod == InclusionSolverSettingsType::InLambda) {
                if(m_globalIterationCounter >= m_settings.m_MinIter && (m_bConverged || m_settings.m_bComputeResidual) ) {
                    nodeData.m_bConverged = Numerics::cancelCriteriaValue(nodeData.m_LambdaBack,
                                            nodeData.m_LambdaFront,
                                            m_settings.m_AbsTol,
                                            m_settings.m_RelTol,
                                            residual
                                                                         );
                    m_pGraph->m_maxResidual = std::max(residual,m_pGraph->m_maxResidual);
                    if(!nodeData.m_bConverged) {
                        // Set global Converged = false;
                        m_bConverged = false;
                    }
                } else {
                    m_bConverged=false;
                }
            }


            // Save residual and insert into front buffer
            #ifdef RESIDUAL_SORTED_ITERATION
                m_pGraph->m_nodesFrontRes->insert(std::make_pair(residual,&node));
            #endif // RESIDUAL_SORTED_ITERATION

            // Swap Lambdas, but dont swap Velocities...
            //nodeData.m_LambdaBack = nodeData.m_LambdaFront; // necessary if we use doVelocityUpdate function!
            nodeData.swapLambdas(); // faster only switch pointers



        } else {
            ERRORMSG(" You specified a contact model which has not been implemented so far!");
        }
    }

};


/**
* This is a full sor, projects normal and then tangential of one contact consecutive!
*/
template<typename TContactGraph>
class FullSorProxStepNodeVisitor : public SorProxStepNodeVisitor<TContactGraph> {
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
    DEFINE_SORPROXSTEPNODE_BASE

    FullSorProxStepNodeVisitor(const InclusionSolverSettingsType &settings,
                               bool & globalConverged,
                               const unsigned int & globalIterationNeeded,
                               ContactGraphType * graph):
        Base(settings,globalConverged,globalIterationNeeded,graph) {
        this->m_delegate = VisitNodeDelegate::template from_method<  FullSorProxStepNodeVisitor,
              &FullSorProxStepNodeVisitor::visitNode>(this);
    }

    void visitNode(NodeType& node) {
        /* Convergence Criterias are no more checked if the m_bConverged (gloablConverged) flag is already false
           Then the iteration is not converged somewhere, and we need to wait till the next iteration!
        */
        typename ContactGraphType::NodeDataType & nodeData = node.m_nodeData;
        static VectorUBody uCache1,uCache2;
        PREC residual;


        LOGSLLEVEL3_CONTACT(m_pSolverLog, "---> SorProx, Node: " << node.m_nodeNumber <<"====================="<<  std::endl);
        if( nodeData.m_pCollData->m_pBody1->m_eMode == RigidBodyType::BodyMode::SIMULATED  &&  nodeData.m_pCollData->m_pBody2->m_eMode == RigidBodyType::BodyMode::SIMULATED) {
            LOGSLLEVEL3_CONTACT(m_pSolverLog, "\t---> Sim<->Sim Node:"<<  std::endl);
        }


        if( nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCF ||
            nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCFD ) {

            //First normal direction ===================================

            PREC lambda_N = nodeData.m_b(0);
            // FIRST BODY!
            if( nodeData.m_pCollData->m_pBody1->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {
                uCache1 = nodeData.m_u1BufferPtr->m_front;
                lambda_N += nodeData.m_W_body1.transpose().row(0) * uCache1 ;
            }
            // SECOND BODY!
            if( nodeData.m_pCollData->m_pBody2->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {
                uCache2 = nodeData.m_u2BufferPtr->m_front;
                lambda_N += nodeData.m_W_body2.transpose().row(0) * uCache2;
            }

            // Damping
            if(nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCFD) {
                using CMT = typename CONTACTMODELTYPE(ContactModels::Enum::UCFD);
                lambda_N += nodeData.m_LambdaBack(0) * nodeData.m_contactParameter.m_params[CMT::d_NIdx] / m_settings.m_deltaT;
            }
            lambda_N = -nodeData.m_R_i_inv_diag(0) * lambda_N;
            lambda_N += nodeData.m_LambdaBack(0);

            //Prox
            Prox::ProxFunction<ConvexSets::RPlus>::doProxSingle( lambda_N, lambda_N );

            //Apply to bodies
            nodeData.m_LambdaFront(0) = lambda_N;
            PREC deltaLambda_N = lambda_N - nodeData.m_LambdaBack(0); // Delta lambda_N

            if( nodeData.m_pCollData->m_pBody1->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {
                nodeData.m_u1BufferPtr->m_front = uCache1
                                                  + nodeData.m_pCollData->m_pBody1->m_MassMatrixInv_diag.asDiagonal() *
                                                  nodeData.m_W_body1.col(0) * deltaLambda_N;
            }
            if( nodeData.m_pCollData->m_pBody2->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {
                nodeData.m_u2BufferPtr->m_front = uCache2
                                                  + nodeData.m_pCollData->m_pBody2->m_MassMatrixInv_diag.asDiagonal() *
                                                  nodeData.m_W_body2.col(0) * deltaLambda_N;
            }


            // =========================================================

            // Second Tangential direction =============================
            Vector2 lambda_T = nodeData.m_b.template tail<2>();
            // FIRST BODY!
            if( nodeData.m_pCollData->m_pBody1->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {
                lambda_T += nodeData.m_W_body1.transpose().template bottomRows<2>() * nodeData.m_u1BufferPtr->m_front ;
            }
            // SECOND BODY!
            if( nodeData.m_pCollData->m_pBody2->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {
                lambda_T += nodeData.m_W_body2.transpose().template bottomRows<2>() * nodeData.m_u2BufferPtr->m_front;
            }
            // Damping
            if(nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCFD) {
                 using CMT = typename CONTACTMODELTYPE(ContactModels::Enum::UCFD);
                lambda_T += nodeData.m_LambdaBack.template tail<2>() * (nodeData.m_contactParameter.m_params[CMT::d_TIdx] / m_settings.m_deltaT);
            }

            lambda_T = - (nodeData.m_R_i_inv_diag.template tail<2>().asDiagonal() * lambda_T).eval();
            lambda_T += nodeData.m_LambdaBack.template tail<2>();

            //Prox
            using CMT = typename CONTACTMODELTYPE(ContactModels::Enum::UCFD);
            Prox::ProxFunction<ConvexSets::Disk>::doProxSingle( nodeData.m_contactParameter.m_params[CMT::muIdx] * lambda_N, lambda_T );

            nodeData.m_LambdaFront.template tail<2>() =  lambda_T;

            lambda_T = lambda_T - nodeData.m_LambdaBack.template tail<2>(); // Delta lambda_T



            if( nodeData.m_pCollData->m_pBody1->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {

                nodeData.m_u1BufferPtr->m_front +=
                    nodeData.m_pCollData->m_pBody1->m_MassMatrixInv_diag.asDiagonal() * nodeData.m_W_body1.template rightCols<2>() * lambda_T;


                LOGSLLEVEL3_CONTACT(m_pSolverLog,"\t---> nd.u1Front: " << nodeData.m_u1BufferPtr->m_front.transpose() << std::endl);

                if(m_settings.m_eConvergenceMethod == InclusionSolverSettingsType::InVelocityLocal) {
                    if(m_globalIterationCounter >= m_settings.m_MinIter && m_bConverged) {
                        nodeData.m_bConverged  = Numerics::cancelCriteriaValue(uCache1,nodeData.m_u1BufferPtr->m_front,m_settings.m_AbsTol, m_settings.m_RelTol);
                        if(!nodeData.m_bConverged ) {
                            //converged stays false;
                            // Set global Converged = false;
                            m_bConverged = false;
                            *m_pSolverLog << "\t---> m_bConverged = false;"<<std::endl;
                        }
                    } else {
                        m_bConverged=false;
                    }
                } else if(m_settings.m_eConvergenceMethod == InclusionSolverSettingsType::InEnergyLocalMix) {
                    if(m_globalIterationCounter >= m_settings.m_MinIter && m_bConverged) {
                        nodeData.m_bConverged  = Numerics::cancelCriteriaMatrixNorm(      uCache1,
                                                 nodeData.m_pCollData->m_pBody1->m_MassMatrix_diag,
                                                 nodeData.m_LambdaBack,
                                                 nodeData.m_LambdaFront,
                                                 nodeData.m_G_ii,
                                                 m_settings.m_AbsTol,
                                                 m_settings.m_RelTol);
                        if(!nodeData.m_bConverged ) {
                            //converged stays false;
                            // Set global Converged = false;
                            m_bConverged = false;
                        }
                    } else {
                        m_bConverged=false;
                    }
                }

            }
            if( nodeData.m_pCollData->m_pBody2->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {

                nodeData.m_u2BufferPtr->m_front +=
                    nodeData.m_pCollData->m_pBody2->m_MassMatrixInv_diag.asDiagonal()*nodeData.m_W_body2.template rightCols<2>() * lambda_T;


                LOGSLLEVEL3_CONTACT(m_pSolverLog,"\t---> nd.u2Front: " << nodeData.m_u2BufferPtr->m_front.transpose() << std::endl);


                if(m_settings.m_eConvergenceMethod == InclusionSolverSettingsType::InVelocityLocal) {
                    if(m_globalIterationCounter >= m_settings.m_MinIter && m_bConverged) {
                        nodeData.m_bConverged  = Numerics::cancelCriteriaValue(uCache2,
                                                 nodeData.m_u2BufferPtr->m_front,
                                                 m_settings.m_AbsTol,
                                                 m_settings.m_RelTol);
                        if(!nodeData.m_bConverged ) {
                            //converged stays false;
                            // Set global Converged = false;
                            m_bConverged = false;
                        }
                    } else {
                        m_bConverged=false;
                    }

                } else if(m_settings.m_eConvergenceMethod == InclusionSolverSettingsType::InEnergyLocalMix) {
                    if(m_globalIterationCounter >= m_settings.m_MinIter && m_bConverged) {
                        nodeData.m_bConverged  = Numerics::cancelCriteriaMatrixNorm(   uCache2,
                                                 nodeData.m_pCollData->m_pBody2->m_MassMatrix_diag,
                                                 nodeData.m_LambdaBack,
                                                 nodeData.m_LambdaFront,
                                                 nodeData.m_G_ii,
                                                 m_settings.m_AbsTol,
                                                 m_settings.m_RelTol);
                        if(!nodeData.m_bConverged ) {
                            //converged stays false;
                            // Set global Converged = false;
                            m_bConverged = false;
                        }
                    } else {
                        m_bConverged=false;
                    }
                }
            }
            // =========================================================



            LOGSLLEVEL3_CONTACT(m_pSolverLog, "\t---> nd.m_LambdaBack: "  << nodeData.m_LambdaBack.transpose() << std::endl);
            LOGSLLEVEL3_CONTACT(m_pSolverLog, "\t---> nd.m_LambdaFront: " << nodeData.m_LambdaFront.transpose() << std::endl);
            if(Numerics::cancelCriteriaValue(nodeData.m_LambdaBack,nodeData.m_LambdaFront,m_settings.m_AbsTol, m_settings.m_RelTol)) {
                LOGSLLEVEL3_CONTACT(m_pSolverLog , "\t---> Lambda converged"<<std::endl);
            }




            if(m_settings.m_eConvergenceMethod == InclusionSolverSettingsType::InLambda) {
                if(m_globalIterationCounter >= m_settings.m_MinIter && (m_bConverged || m_settings.m_bComputeResidual) ) {
                    nodeData.m_bConverged = Numerics::cancelCriteriaValue(nodeData.m_LambdaBack,
                                            nodeData.m_LambdaFront,
                                            m_settings.m_AbsTol,
                                            m_settings.m_RelTol,
                                            residual
                                                                         );
                    m_pGraph->m_maxResidual = std::max(residual,m_pGraph->m_maxResidual);
                    if(!nodeData.m_bConverged) {
                        // Set global Converged = false;
                        m_bConverged = false;
                    }
                } else {
                    m_bConverged=false;
                }
            }


            // Swap Lambdas, but dont swap Velocities...
            //nodeData.m_LambdaBack = nodeData.m_LambdaFront; // necessary if we use doVelocityUpdate function!
            nodeData.swapLambdas(); // faster only switch pointers



        } else {
            ERRORMSG(" You specified a contact model which has not been implemented so far!");
        }
    }

};

/**
* This is a normal SOR Prox, projects normal directions of one contact consecutive!
*/
template<typename TContactGraph>
class NormalSorProxStepNodeVisitor : public SorProxStepNodeVisitor<TContactGraph> {
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
    DEFINE_SORPROXSTEPNODE_BASE

    NormalSorProxStepNodeVisitor(const InclusionSolverSettingsType &settings,
                               bool & globalConverged,
                               const unsigned int & globalIterationNeeded,
                               ContactGraphType * graph):
        Base(settings,globalConverged,globalIterationNeeded,graph) {
        this->m_delegate = VisitNodeDelegate::template from_method<  NormalSorProxStepNodeVisitor,
              &NormalSorProxStepNodeVisitor::visitNode >(this);
    }

    void setLastUpdate(bool b){
        m_lastUpdate = b;
    }

    void visitNode(NodeType& node) {
        /* Convergence Criterias are no more checked if the m_bConverged (gloablConverged) flag is already false
           Then the iteration is not converged somewhere, and we need to wait till the next iteration!
        */
        typename ContactGraphType::NodeDataType & nodeData = node.m_nodeData;
        static VectorUBody uCache1,uCache2;
        PREC residual;


        LOGSLLEVEL3_CONTACT(m_pSolverLog, "---> SorProx, Node: " << node.m_nodeNumber <<"====================="<<  std::endl);
        if( nodeData.m_pCollData->m_pBody1->m_eMode == RigidBodyType::BodyMode::SIMULATED  &&  nodeData.m_pCollData->m_pBody2->m_eMode == RigidBodyType::BodyMode::SIMULATED) {
            LOGSLLEVEL3_CONTACT(m_pSolverLog, "\t---> Sim<->Sim Node:"<<  std::endl);
        }


        if( nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCF ||
            nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCFD ) {

            //Only normal direction ===================================

            PREC lambda_N = nodeData.m_b(0);
            // FIRST BODY!
            if( nodeData.m_pCollData->m_pBody1->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {
                uCache1 = nodeData.m_u1BufferPtr->m_front;
                lambda_N += nodeData.m_W_body1.transpose().row(0) * uCache1 ;
            }
            // SECOND BODY!
            if( nodeData.m_pCollData->m_pBody2->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {
                uCache2 = nodeData.m_u2BufferPtr->m_front;
                lambda_N += nodeData.m_W_body2.transpose().row(0) * uCache2;
            }

            // Damping
            if(nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCFD) {
                using CMT = typename CONTACTMODELTYPE(ContactModels::Enum::UCFD);
                lambda_N += nodeData.m_LambdaBack(0) * nodeData.m_contactParameter.m_params[CMT::d_NIdx] / m_settings.m_deltaT;
            }
            lambda_N = -nodeData.m_R_i_inv_diag(0) * lambda_N;
            lambda_N += nodeData.m_LambdaBack(0);

            //Prox
            Prox::ProxFunction<ConvexSets::RPlus>::doProxSingle( lambda_N, lambda_N );

            //Apply to bodies

            PREC deltaLambda_N = lambda_N - nodeData.m_LambdaBack(0); // Delta lambda_N

            if( nodeData.m_pCollData->m_pBody1->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {
                nodeData.m_u1BufferPtr->m_front = uCache1
                                                  + nodeData.m_pCollData->m_pBody1->m_MassMatrixInv_diag.asDiagonal() *
                                                  nodeData.m_W_body1.col(0) * deltaLambda_N;
            }
            if( nodeData.m_pCollData->m_pBody2->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {
                nodeData.m_u2BufferPtr->m_front = uCache2
                                                  + nodeData.m_pCollData->m_pBody2->m_MassMatrixInv_diag.asDiagonal() *
                                                  nodeData.m_W_body2.col(0) * deltaLambda_N;
            }

            if(m_lastUpdate){
                nodeData.m_LambdaFront(0) = lambda_N;
            }else{
                nodeData.m_LambdaBack(0) = lambda_N;
            }
            // =========================================================

            // Apply the tangential step node visitor now

        } else {
            ERRORMSG(" You specified a contact model which has not been implemented so far!");
        }
    }

    bool m_lastUpdate = false;

};

/**
* This is a tangential SOR Prox, projects tangential directions of one contact consecutive!
*/
template<typename TContactGraph>
class TangentialSorProxStepNodeVisitor : public SorProxStepNodeVisitor<TContactGraph> {
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
    DEFINE_SORPROXSTEPNODE_BASE

    TangentialSorProxStepNodeVisitor(const InclusionSolverSettingsType &settings,
                               bool & globalConverged,
                               const unsigned int & globalIterationNeeded,
                               ContactGraphType * graph):
        Base(settings,globalConverged,globalIterationNeeded,graph) {
        this->m_delegate = VisitNodeDelegate::template from_method<  TangentialSorProxStepNodeVisitor,
              &TangentialSorProxStepNodeVisitor::visitNode>(this);
    }

    void visitNode(NodeType& node) {
        /* Convergence Criterias are no more checked if the m_bConverged (gloablConverged) flag is already false
           Then the iteration is not converged somewhere, and we need to wait till the next iteration!
        */
        typename ContactGraphType::NodeDataType & nodeData = node.m_nodeData;
        static VectorUBody uCache1,uCache2;
        PREC residual;


        LOGSLLEVEL3_CONTACT(m_pSolverLog, "---> SorProx, Node: " << node.m_nodeNumber <<"====================="<<  std::endl);
        if( nodeData.m_pCollData->m_pBody1->m_eMode == RigidBodyType::BodyMode::SIMULATED  &&  nodeData.m_pCollData->m_pBody2->m_eMode == RigidBodyType::BodyMode::SIMULATED) {
            LOGSLLEVEL3_CONTACT(m_pSolverLog, "\t---> Sim<->Sim Node:"<<  std::endl);
        }


        if( nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCF ||
                nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCFD ) {


            // Second Tangential direction =============================
            Vector2 lambda_T = nodeData.m_b.template tail<2>();
            // FIRST BODY!
            if( nodeData.m_pCollData->m_pBody1->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {
                lambda_T += nodeData.m_W_body1.transpose().template bottomRows<2>() * nodeData.m_u1BufferPtr->m_front ;
            }
            // SECOND BODY!
            if( nodeData.m_pCollData->m_pBody2->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {
                lambda_T += nodeData.m_W_body2.transpose().template bottomRows<2>() * nodeData.m_u2BufferPtr->m_front;
            }
            // Damping
            if(nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCFD) {
                 using CMT = typename CONTACTMODELTYPE(ContactModels::Enum::UCFD);
                lambda_T += nodeData.m_LambdaBack.template tail<2>() * (nodeData.m_contactParameter.m_params[CMT::d_TIdx] / m_settings.m_deltaT);
            }

            lambda_T = - (nodeData.m_R_i_inv_diag.template tail<2>().asDiagonal() * lambda_T).eval();
            lambda_T += nodeData.m_LambdaBack.template tail<2>();

            //Prox
            using CMT = typename CONTACTMODELTYPE(ContactModels::Enum::UCFD);
            Prox::ProxFunction<ConvexSets::Disk>::doProxSingle( nodeData.m_contactParameter.m_params[CMT::muIdx] * nodeData.m_LambdaFront(0), lambda_T );

            nodeData.m_LambdaFront.template tail<2>() =  lambda_T;

            lambda_T = lambda_T - nodeData.m_LambdaBack.template tail<2>(); // Delta lambda_T


            if( nodeData.m_pCollData->m_pBody1->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {

                nodeData.m_u1BufferPtr->m_front +=
                    nodeData.m_pCollData->m_pBody1->m_MassMatrixInv_diag.asDiagonal() * nodeData.m_W_body1.template rightCols<2>() * lambda_T;


                LOGSLLEVEL3_CONTACT(m_pSolverLog,"\t---> nd.u1Front: " << nodeData.m_u1BufferPtr->m_front.transpose() << std::endl);

                if(m_settings.m_eConvergenceMethod == InclusionSolverSettingsType::InVelocityLocal) {
                    if(m_globalIterationCounter >= m_settings.m_MinIter && m_bConverged) {
                        nodeData.m_bConverged  = Numerics::cancelCriteriaValue(uCache1,nodeData.m_u1BufferPtr->m_front,m_settings.m_AbsTol, m_settings.m_RelTol);
                        if(!nodeData.m_bConverged ) {
                            //converged stays false;
                            // Set global Converged = false;
                            m_bConverged = false;
                            *m_pSolverLog << "\t---> m_bConverged = false;"<<std::endl;
                        }
                    } else {
                        m_bConverged=false;
                    }
                } else if(m_settings.m_eConvergenceMethod == InclusionSolverSettingsType::InEnergyLocalMix) {
                    if(m_globalIterationCounter >= m_settings.m_MinIter && m_bConverged) {
                        nodeData.m_bConverged  = Numerics::cancelCriteriaMatrixNorm(      uCache1,
                                                 nodeData.m_pCollData->m_pBody1->m_MassMatrix_diag,
                                                 nodeData.m_LambdaBack,
                                                 nodeData.m_LambdaFront,
                                                 nodeData.m_G_ii,
                                                 m_settings.m_AbsTol,
                                                 m_settings.m_RelTol);
                        if(!nodeData.m_bConverged ) {
                            //converged stays false;
                            // Set global Converged = false;
                            m_bConverged = false;
                        }
                    } else {
                        m_bConverged=false;
                    }
                }

            }
            if( nodeData.m_pCollData->m_pBody2->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {

                nodeData.m_u2BufferPtr->m_front +=
                    nodeData.m_pCollData->m_pBody2->m_MassMatrixInv_diag.asDiagonal()*nodeData.m_W_body2.template rightCols<2>() * lambda_T;


                LOGSLLEVEL3_CONTACT(m_pSolverLog,"\t---> nd.u2Front: " << nodeData.m_u2BufferPtr->m_front.transpose() << std::endl);


                if(m_settings.m_eConvergenceMethod == InclusionSolverSettingsType::InVelocityLocal) {
                    if(m_globalIterationCounter >= m_settings.m_MinIter && m_bConverged) {
                        nodeData.m_bConverged  = Numerics::cancelCriteriaValue(uCache2,
                                                 nodeData.m_u2BufferPtr->m_front,
                                                 m_settings.m_AbsTol,
                                                 m_settings.m_RelTol);
                        if(!nodeData.m_bConverged ) {
                            //converged stays false;
                            // Set global Converged = false;
                            m_bConverged = false;
                        }
                    } else {
                        m_bConverged=false;
                    }

                } else if(m_settings.m_eConvergenceMethod == InclusionSolverSettingsType::InEnergyLocalMix) {
                    if(m_globalIterationCounter >= m_settings.m_MinIter && m_bConverged) {
                        nodeData.m_bConverged  = Numerics::cancelCriteriaMatrixNorm(   uCache2,
                                                 nodeData.m_pCollData->m_pBody2->m_MassMatrix_diag,
                                                 nodeData.m_LambdaBack,
                                                 nodeData.m_LambdaFront,
                                                 nodeData.m_G_ii,
                                                 m_settings.m_AbsTol,
                                                 m_settings.m_RelTol);
                        if(!nodeData.m_bConverged ) {
                            //converged stays false;
                            // Set global Converged = false;
                            m_bConverged = false;
                        }
                    } else {
                        m_bConverged=false;
                    }
                }
            }
            // =========================================================



            LOGSLLEVEL3_CONTACT(m_pSolverLog, "\t---> nd.m_LambdaBack: "  << nodeData.m_LambdaBack.transpose() << std::endl);
            LOGSLLEVEL3_CONTACT(m_pSolverLog, "\t---> nd.m_LambdaFront: " << nodeData.m_LambdaFront.transpose() << std::endl);
            if(Numerics::cancelCriteriaValue(nodeData.m_LambdaBack,nodeData.m_LambdaFront,m_settings.m_AbsTol, m_settings.m_RelTol)) {
                LOGSLLEVEL3_CONTACT(m_pSolverLog , "\t---> Lambda converged"<<std::endl);
            }




            if(m_settings.m_eConvergenceMethod == InclusionSolverSettingsType::InLambda) {
                if(m_globalIterationCounter >= m_settings.m_MinIter && (m_bConverged || m_settings.m_bComputeResidual) ) {
                    nodeData.m_bConverged = Numerics::cancelCriteriaValue(nodeData.m_LambdaBack,
                                            nodeData.m_LambdaFront,
                                            m_settings.m_AbsTol,
                                            m_settings.m_RelTol,
                                            residual
                                                                         );
                    m_pGraph->m_maxResidual = std::max(residual,m_pGraph->m_maxResidual);
                    if(!nodeData.m_bConverged) {
                        // Set global Converged = false;
                        m_bConverged = false;
                    }
                } else {
                    m_bConverged=false;
                }
            }


            // Swap Lambdas, but dont swap Velocities...
            //nodeData.m_LambdaBack = nodeData.m_LambdaFront; // necessary if we use doVelocityUpdate function!
            nodeData.swapLambdas(); // faster only switch pointers



        } else {
            ERRORMSG(" You specified a contact model which has not been implemented so far!");
        }
    }

};

/**
@brief Visitor for class ContactGraph<TRigidBody,ContactGraphMode::ForIteration>
*/
template<typename TContactGraph>
class SorProxInitNodeVisitor {
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

    using ContactGraphType = TContactGraph;
    using NodeDataType = typename ContactGraphType::NodeDataType;
    using EdgeDataType = typename ContactGraphType::EdgeDataType;
    using EdgeType = typename ContactGraphType::EdgeType;
    using NodeType = typename ContactGraphType::NodeType;

    SorProxInitNodeVisitor(const InclusionSolverSettingsType &settings): m_alpha(1), m_settings(settings)
    {}

    void setLog(Logging::Log * solverLog) {
        m_pSolverLog = solverLog;
    }

    // Set Sor Prox parameter, before calling visitNode
    void setParams(PREC alpha) {
        m_alpha = alpha;
    }


    template<int bodyNr>
    inline void computeW_UCF(NodeDataType & nodeData) {

            static Matrix33 I_r_SiCi_hat = Matrix33::Zero();
            static Matrix33 I_Jacobi_2; // this is the second part of the Jacobi;
            static const CollisionData * pCollData;

            pCollData = nodeData.m_pCollData;

            if(bodyNr == 1) {
                //Set matrix size!
                nodeData.m_W_body1.setZero(NDOFuBody, ContactModels::getLambdaDim(ContactModels::Enum::UCF));

                updateSkewSymmetricMatrix<>( pCollData->m_r_S1C1, I_r_SiCi_hat);
                I_Jacobi_2 = ( nodeData.m_pCollData->m_pBody1->m_A_IK.transpose() * I_r_SiCi_hat );
                // N direction =================================================
                nodeData.m_W_body1.col(0).template head<3>() = - pCollData->m_cFrame.m_e_z; // I frame

                nodeData.m_W_body1.col(0).template tail<3>() = - I_Jacobi_2 * pCollData->m_cFrame.m_e_z;

                // T1 direction =================================================
                nodeData.m_W_body1.col(1).template head<3>() = - pCollData->m_cFrame.m_e_x; // I frame
                nodeData.m_W_body1.col(1).template tail<3>() = - I_Jacobi_2 * pCollData->m_cFrame.m_e_x;

                // T2 direction =================================================
                nodeData.m_W_body1.col(2).template head<3>() = - pCollData->m_cFrame.m_e_y; // I frame
                nodeData.m_W_body1.col(2).template tail<3>() = - I_Jacobi_2 * pCollData->m_cFrame.m_e_y;
            } else {
                //Set matrix size!
                nodeData.m_W_body2.setZero(NDOFuBody, ContactModels::getLambdaDim(ContactModels::Enum::UCF));

                updateSkewSymmetricMatrix<>( pCollData->m_r_S2C2, I_r_SiCi_hat);
                I_Jacobi_2 = ( nodeData.m_pCollData->m_pBody2->m_A_IK.transpose() * I_r_SiCi_hat );
                // N direction =================================================
                nodeData.m_W_body2.col(0).template head<3>() =  pCollData->m_cFrame.m_e_z; // I frame
                nodeData.m_W_body2.col(0).template tail<3>() =  I_Jacobi_2 * pCollData->m_cFrame.m_e_z;

                // T1 direction =================================================
                nodeData.m_W_body2.col(1).template head<3>() =  pCollData->m_cFrame.m_e_x; // I frame
                nodeData.m_W_body2.col(1).template tail<3>() =  I_Jacobi_2 * pCollData->m_cFrame.m_e_x;

                // T2 direction =================================================
                nodeData.m_W_body2.col(2).template head<3>() =  pCollData->m_cFrame.m_e_y; // I frame
                nodeData.m_W_body2.col(2).template tail<3>() =  I_Jacobi_2 * pCollData->m_cFrame.m_e_y;
            }
    }


    void visitNode(NodeType & node) {
        auto & nodeData = node.m_nodeData;

        // Initialize for UCF Contact models
        if( nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCF ||
            nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCFD ||
            nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCFDD  ) {

            using CMT = typename CONTACTMODELTYPE(ContactModels::Enum::UCF);
            const unsigned int dimSet = ContactModels::getLambdaDim(ContactModels::Enum::UCF);

            nodeData.m_b.setZero(dimSet);
            nodeData.m_LambdaBack.setZero(dimSet);
            nodeData.m_LambdaFront.setZero(dimSet);
            nodeData.m_R_i_inv_diag.setZero(dimSet);
            nodeData.m_G_ii.setZero(dimSet,dimSet);

            // Compute generalized force directions W
            auto state = nodeData.m_pCollData->m_pBody1->m_eMode;
            if(  state == RigidBodyType::BodyMode::SIMULATED){
                computeW_UCF<1>(nodeData);
            }else if(state == RigidBodyType::BodyMode::ANIMATED){
                // Contact goes into xi_N, xi_T
                ASSERTMSG(false,"RigidBody<TLayoutConfig>::ANIMATED objects have not been implemented correctly so far!");
            }
            state = nodeData.m_pCollData->m_pBody2->m_eMode;
            if(  state == RigidBodyType::BodyMode::SIMULATED){
                computeW_UCF<2>(nodeData);
            }else if(state == RigidBodyType::BodyMode::ANIMATED){
                // Contact goes into xi_N, xi_T
                ASSERTMSG(false,"RigidBody<TLayoutConfig>::ANIMATED objects have not been implemented correctly so far!");
            }

            // Get lambda from percussion pool otherwise set to zero
            // TODO
            nodeData.m_LambdaBack.setZero();

            // (1+e)*xi -> b
            nodeData.m_b = (nodeData.m_eps.array() + 1).matrix().asDiagonal() * nodeData.m_chi;

            // u_0 , calculate const b
            // First Body
            if(nodeData.m_pCollData->m_pBody1->m_eMode == RigidBodyType::BodyMode::SIMULATED) {

                // m_front is zero here-> see DynamicsSystem sets it to zero!
                nodeData.m_u1BufferPtr->m_front +=  nodeData.m_pCollData->m_pBody1->m_MassMatrixInv_diag.asDiagonal() * (nodeData.m_W_body1 * nodeData.m_LambdaBack );
                /// + initial values M^⁻1 W lambda0 from percussion pool

                nodeData.m_b += nodeData.m_eps.asDiagonal() * nodeData.m_W_body1.transpose() * nodeData.m_u1BufferPtr->m_back /* m_u_s */ ;
                nodeData.m_G_ii += nodeData.m_W_body1.transpose() * nodeData.m_pCollData->m_pBody1->m_MassMatrixInv_diag.asDiagonal() * nodeData.m_W_body1 ;
            }
            // SECOND BODY!
            if(nodeData.m_pCollData->m_pBody2->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {

                // m_front is zero here-> see DynamicsSystem sets it to zero!
                nodeData.m_u2BufferPtr->m_front +=   nodeData.m_pCollData->m_pBody2->m_MassMatrixInv_diag.asDiagonal() * (nodeData.m_W_body2 * nodeData.m_LambdaBack );
                /// + initial values M^⁻1 W lambda0 from percussion pool

                nodeData.m_b += nodeData.m_eps.asDiagonal() * nodeData.m_W_body2.transpose() *  nodeData.m_u2BufferPtr->m_back;
                nodeData.m_G_ii += nodeData.m_W_body2.transpose() * nodeData.m_pCollData->m_pBody2->m_MassMatrixInv_diag.asDiagonal() * nodeData.m_W_body2 ;
            }

            if(nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCFD) {
                using CMT = typename CONTACTMODELTYPE(ContactModels::Enum::UCFD);
                Vector3 d(  nodeData.m_contactParameter.m_params[CMT::d_NIdx], //d_N
                            nodeData.m_contactParameter.m_params[CMT::d_TIdx], //d_T
                            nodeData.m_contactParameter.m_params[CMT::d_TIdx]);//d_T
                nodeData.m_G_ii.diagonal() += 1.0/m_settings.m_deltaT*d;
            } else if(nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCFDD) {
                using CMT = typename CONTACTMODELTYPE(ContactModels::Enum::UCFDD);
                Vector3 d(  nodeData.m_contactParameter.m_params[CMT::d_NIdx],    //d_N
                            nodeData.m_contactParameter.m_params[CMT::d_TfixIdx], //d_TFix
                            nodeData.m_contactParameter.m_params[CMT::d_TfixIdx]);//d_TFix
                nodeData.m_G_ii.diagonal() += 1.0/m_settings.m_deltaT*d;
            }

            // Calculate R_ii

            if(m_settings.m_eMethod == InclusionSolverSettingsType::SOR_CONTACT_DS){
                // De Saxe Formulation, only one r parameter because friction cone
                if(m_settings.m_RStrategy == InclusionSolverSettingsType::RSTRATEGY_SUM){
                   nodeData.m_R_i_inv_diag.setConstant( m_alpha / nodeData.m_G_ii.diagonal().sum() );
                }
                else if(m_settings.m_RStrategy == InclusionSolverSettingsType::RSTRATEGY_SUM2){
                   nodeData.m_R_i_inv_diag.setConstant( m_alpha / (nodeData.m_G_ii.diagonal().sum() *3) );
                }
                else if(m_settings.m_RStrategy == InclusionSolverSettingsType::RSTRATEGY_MAX){
                   nodeData.m_R_i_inv_diag.setConstant( m_alpha / nodeData.m_G_ii.diagonal().maxCoeff() );
                }else{
                    ERRORMSG(" You specified a R-Matrix strategy which has not been implemented so far!");
                }

            }else{
                // Alart Curnier Formulation
                // De Saxe Formulation, only one r parameter because friction cone
                if(m_settings.m_RStrategy == InclusionSolverSettingsType::RSTRATEGY_SUM){

                   nodeData.m_R_i_inv_diag(0) = m_alpha / nodeData.m_G_ii(0,0);
                   nodeData.m_R_i_inv_diag.template tail<2>().setConstant( m_alpha / nodeData.m_G_ii.diagonal().template tail<2>().sum() );

                }else if(m_settings.m_RStrategy == InclusionSolverSettingsType::RSTRATEGY_MAX){
                    // Take also offdiagonal values for lambda_N
                    //nodeData.m_R_i_inv_diag(0) = m_alpha / std::max(std::max(nodeData.m_G_ii(0,0), nodeData.m_mu(0)*nodeData.m_G_ii(0,1)), nodeData.m_mu(0)*nodeData.m_G_ii(0,2));
                    // Take only diagonal
                    //std::cout << " nodeData.m_G_ii(0,0) " << nodeData.m_G_ii(0,0) << std::endl;
                    nodeData.m_R_i_inv_diag(0) = m_alpha / nodeData.m_G_ii(0,0);
                    PREC r_T = m_alpha / (nodeData.m_G_ii.diagonal().template tail<2>().maxCoeff());
                    nodeData.m_R_i_inv_diag(1) = r_T;
                    nodeData.m_R_i_inv_diag(2) = r_T;

                }else{
                    ERRORMSG(" You specified a R-Matrix strategy which has not been implemented so far!");
                }
            }



            LOGSLLEVEL3_CONTACT(m_pSolverLog, "\t ---> nd.m_b: "<< nodeData.m_b.transpose() <<std::endl
                << "\t ---> nd.m_G_ii: "<<std::endl<< nodeData.m_G_ii <<std::endl
                << "\t ---> nd.m_R_i_inv_diag: "<< nodeData.m_R_i_inv_diag.transpose() <<std::endl;);



            LOGSLLEVEL3_CONTACT(m_pSolverLog,  "\t ---> nd.m_mu: "<< nodeData.m_contactParameter.m_params[CMT::muIdx] <<std::endl;);

        } else {
            ERRORMSG(" You specified a contact model which has not been implemented so far!");
        }

    }

private:
    Logging::Log * m_pSolverLog;
    PREC m_alpha;
    InclusionSolverSettingsType m_settings;
};

#endif // ContactGraphVisitors_hpp