#ifndef ContactGraphVisitorMPI_hpp
#define ContactGraphVisitorMPI_hpp

#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include "SimpleLogger.hpp"

#include "ContactModels.hpp"
#include "ProxFunctions.hpp"
#include "InclusionSolverSettings.hpp"

/**
@brief Visitor for class ContactGraph
*/
template<typename TContactGraph>
class SorProxStepNodeVisitor{
public:

    DEFINE_RIGIDBODY_CONFIG_TYPES

    typedef TContactGraph ContactGraphType;
    typedef typename ContactGraphType::NodeDataType NodeDataType;
    typedef typename ContactGraphType::EdgeDataType EdgeDataType;
    typedef typename ContactGraphType::EdgeType EdgeType;
    typedef typename ContactGraphType::NodeType NodeType;

    SorProxStepNodeVisitor(const InclusionSolverSettings &settings,
                           bool & globalConverged, const unsigned int & globalIterationNeeded):
            m_Settings(settings),m_bConverged(globalConverged),
            m_iterationsNeeded(globalIterationNeeded)
    {}

    void setLog(Logging::Log * solverLog){
        m_pSolverLog = solverLog;
    }

    void visitNode(NodeType& node){
        /* Convergence Criterias are no more checked if the m_bConverged (gloablConverged) flag is already false
           Then the iteration is not converged somewhere, and we need to wait till the next iteration!
        */
        typename ContactGraphType::NodeDataType & nodeData = node.m_nodeData;
        static VectorDyn uCache1,uCache2;

        #if CoutLevelSolverWhenContact>2
            LOG(m_pSolverLog, "---> SorProx, Normal Node: " << node.m_nodeNumber <<"====================="<<  std::endl);
        #endif

        if( nodeData.m_eContactModel == ContactModels::NCF_ContactModel ) {


            // Init the prox value
            nodeData.m_LambdaFront = nodeData.m_b;

            // FIRST BODY!
            if( nodeData.m_pCollData->m_pBody1->m_eState == RigidBodyType::BodyState::SIMULATED ) {
                nodeData.m_LambdaFront += nodeData.m_W_body1.transpose() * nodeData.m_u1BufferPtr->m_front ;
            }
            // SECOND BODY!
            if( nodeData.m_pCollData->m_pBody2->m_eState == RigidBodyType::BodyState::SIMULATED ) {
                nodeData.m_LambdaFront += nodeData.m_W_body2.transpose() * nodeData.m_u2BufferPtr->m_front;
            }

            nodeData.m_LambdaFront = -(nodeData.m_R_i_inv_diag.asDiagonal() * nodeData.m_LambdaFront).eval(); //No alias due to diagonal!!! (if normal matrix multiplication there is aliasing!
            nodeData.m_LambdaFront += nodeData.m_LambdaBack;
            //Prox

            Prox::ProxFunction<ConvexSets::RPlusAndDisk>::doProxSingle(
                nodeData.m_mu(0),
                nodeData.m_LambdaFront.template head<ContactModels::NormalAndCoulombFrictionContactModel::ConvexSet::Dimension>()
            );

#if CoutLevelSolverWhenContact>2
            LOG(m_pSolverLog, "\t---> nd.m_LambdaBack: "  << nodeData.m_LambdaBack.transpose() << std::endl);
            LOG(m_pSolverLog, "\t---> nd.m_LambdaFront: " << nodeData.m_LambdaFront.transpose() << std::endl);
            if(Numerics::cancelCriteriaValue(nodeData.m_LambdaBack,nodeData.m_LambdaFront,m_Settings.m_AbsTol, m_Settings.m_RelTol)){
              *m_pSolverLog <<"\t---> Lambda converged"<<std::endl;
            }
#endif

            // u_k+1 = u_k + M^-1 W (lambda_k+1 - lambda_k)
            // FIRST BODY!
            if( nodeData.m_pCollData->m_pBody1->m_eState == RigidBodyType::BodyState::SIMULATED ) {
                uCache1 = nodeData.m_u1BufferPtr->m_front;
                nodeData.m_u1BufferPtr->m_front = nodeData.m_u1BufferPtr->m_front + nodeData.m_pCollData->m_pBody1->m_MassMatrixInv_diag.asDiagonal() * nodeData.m_W_body1 * ( nodeData.m_LambdaFront - nodeData.m_LambdaBack );

               #if CoutLevelSolverWhenContact>2
                LOG(m_pSolverLog,"\t---> nd.u1Front: " << nodeData.m_u1BufferPtr->m_front.transpose() << std::endl);
               #endif


                if(m_Settings.m_eConvergenceMethod == InclusionSolverSettings::InVelocityLocal) {
                    if(m_iterationsNeeded >= m_Settings.m_MinIter && m_bConverged) {
                        nodeData.m_bConverged  = Numerics::cancelCriteriaValue(uCache1,nodeData.m_u1BufferPtr->m_front,m_Settings.m_AbsTol, m_Settings.m_RelTol);
                        if(!nodeData.m_bConverged ) {
                            //converged stays false;
                            // Set global Converged = false;
                            m_bConverged = false;
                            *m_pSolverLog << "m_bConverged = false;"<<std::endl;
                        }
                    } else {
                        m_bConverged=false;
                    }
                }else if(m_Settings.m_eConvergenceMethod == InclusionSolverSettings::InEnergyLocalMix){
                    if(m_iterationsNeeded >= m_Settings.m_MinIter && m_bConverged) {
                        nodeData.m_bConverged  = Numerics::cancelCriteriaMatrixNorm(   uCache1,
                                                                          nodeData.m_pCollData->m_pBody1->m_MassMatrix_diag,
                                                                          nodeData.m_LambdaBack,
                                                                          nodeData.m_LambdaFront,
                                                                          nodeData.m_G_ii,
                                                                          m_Settings.m_AbsTol,
                                                                          m_Settings.m_RelTol);
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
            if( nodeData.m_pCollData->m_pBody2->m_eState == RigidBodyType::BodyState::SIMULATED ) {
                uCache2 = nodeData.m_u2BufferPtr->m_front;
                nodeData.m_u2BufferPtr->m_front = nodeData.m_u2BufferPtr->m_front  + nodeData.m_pCollData->m_pBody2->m_MassMatrixInv_diag.asDiagonal() * nodeData.m_W_body2 * ( nodeData.m_LambdaFront - nodeData.m_LambdaBack );

                #if CoutLevelSolverWhenContact>2
                LOG(m_pSolverLog,"\t---> nd.u2Front: " << nodeData.m_u1BufferPtr->m_front.transpose() << std::endl);
                #endif

                if(m_Settings.m_eConvergenceMethod == InclusionSolverSettings::InVelocityLocal) {
                    if(m_iterationsNeeded >= m_Settings.m_MinIter && m_bConverged) {
                        nodeData.m_bConverged  = Numerics::cancelCriteriaValue(uCache2,
                                                                  nodeData.m_u2BufferPtr->m_front,
                                                                  m_Settings.m_AbsTol,
                                                                  m_Settings.m_RelTol);
                        if(!nodeData.m_bConverged ) {
                            //converged stays false;
                            // Set global Converged = false;
                            m_bConverged = false;
                        }
                    } else {
                        m_bConverged=false;
                    }

                }else if(m_Settings.m_eConvergenceMethod == InclusionSolverSettings::InEnergyLocalMix){
                    if(m_iterationsNeeded >= m_Settings.m_MinIter && m_bConverged) {
                        nodeData.m_bConverged  = Numerics::cancelCriteriaMatrixNorm(   uCache2,
                                                                          nodeData.m_pCollData->m_pBody2->m_MassMatrix_diag,
                                                                          nodeData.m_LambdaBack,
                                                                          nodeData.m_LambdaFront,
                                                                          nodeData.m_G_ii,
                                                                          m_Settings.m_AbsTol,
                                                                          m_Settings.m_RelTol);
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

            if(m_Settings.m_eConvergenceMethod == InclusionSolverSettings::InLambda) {
                if(m_iterationsNeeded >= m_Settings.m_MinIter && m_bConverged) {
                    nodeData.m_bConverged = Numerics::cancelCriteriaValue(nodeData.m_LambdaBack,nodeData.m_LambdaFront,m_Settings.m_AbsTol, m_Settings.m_RelTol);
                    if(!nodeData.m_bConverged) {
                        //converged stays false;
                        // Set global Converged = false;
                        m_bConverged = false;
                    }
                } else {
                    m_bConverged=false;
                }
            }

            // Swap Lambdas, but dont swap Velocities...
            // Swap velocities when we finished ONE Sor Prox Iteration! (very important!)
            nodeData.swapLambdas();


        } else {
            ASSERTMSG(false," You specified a contact model which has not been implemented so far!");
        }
    }

private:
    Logging::Log * m_pSolverLog;
    const InclusionSolverSettings & m_Settings;
    bool & m_bConverged; ///< Access to global flag for cancelation criteria
    const unsigned int & m_iterationsNeeded; ///< Access to global iteration counter

};


/**
@brief Visitor for class ContactGraph
*/
template<typename TContactGraph>
class SorProxStepSplitNodeVisitor{
public:

    DEFINE_RIGIDBODY_CONFIG_TYPES

    typedef TContactGraph ContactGraphType;


    typedef typename ContactGraphType::SplitBodyNodeDataType NodeType;

    SorProxStepSplitNodeVisitor(const InclusionSolverSettings &settings, bool & globalConverged, const unsigned int & globalIterationNeeded):
                           m_Settings(settings),m_bConverged(globalConverged),
                           m_iterationsNeeded(globalIterationNeeded)
    {}

    void setLog(Logging::Log * solverLog){
        m_pSolverLog = solverLog;
    }

    void visitNode(NodeType& node){
        /* Convergence Criterias are no more checked if the m_bConverged (gloablConverged) flag is already false
           Then the iteration is not converged somewhere, and we need to wait till the next iteration!
        */
        #if CoutLevelSolverWhenContact>2
            LOG(m_pSolverLog, "---> SorProx, Billateral Node: ====================="<<  std::endl << " local body id: "
                << RigidBodyId::getBodyIdString(node.m_pBody->m_id) );
        #endif

        auto mult = node.getMultiplicity();
        // Calculate the exact values for the billateral split nodes
        node.m_uBack.template head<NDOFuObj>() = node.m_pBody->m_pSolverData->m_uBuffer.m_front;
        // Build gamma = [u1-u2, u2-u3, u3-u4,..., un-1- un]
        node.m_gamma =   node.m_uBack.head(NDOFuObj*node.m_nLambdas) - node.m_uBack.segment(NDOFuObj, NDOFuObj*node.m_nLambdas);

        #if CoutLevelSolverWhenContact>2
            LOG(m_pSolverLog, "\t---> nd.m_gamma: " << node.m_gamma.transpose() );
        #endif
        // calculate L⁻¹*gamma = Lambda, where L⁻¹ is the matrix choosen by the multiplicity

        node.m_LambdaFront = node.getLInvMatrix() * node.m_gamma;

        #if CoutLevelSolverWhenContact>2
            LOG(m_pSolverLog, "\t---> nd.m_LambdaFront: " << node.m_LambdaFront.transpose() );
        #endif

        //Propagate billateral forces to velocities:
        // The sign is contained in the m_multiplicityWeights vector
        // u_G_End = uBack + M_G⁻¹ * W_M * Lambda_M


        node.m_uFront.segement(NDOFuObj,NDOFuObj*node.m_nLambdas) = node.m_LambdaFront;
        node.m_uFront.template segement(NDOFuObj,NDOFuObj*node.m_nLambdas) -= node.m_LambdaFront;
        for(int i = 0; i<mult; i++){
            node.m_uFront.segement(NDOFuObj*i,NDOFuObj) *= 1.0 / node.m_multiplicityWeights(i);
        }
        node.m_uFront  +=  node.m_uBack;

        #if CoutLevelSolverWhenContact>2
            LOG(m_pSolverLog, "\t---> nd.m_uFront: " << node.m_uFront.transpose() );
        #endif

        // Because we solve this billateral contact directly, we are converged for this node!
        // no change of the flag m_bConverged
    }

private:
    Logging::Log * m_pSolverLog;
    const InclusionSolverSettings & m_Settings;
    bool & m_bConverged; ///< Access to global flag for cancelation criteria
    const unsigned int & m_iterationsNeeded; ///< Access to global iteration counter

};




/**
@brief Visitor for class ContactGraph
*/
template<typename TContactGraph>
class SorProxInitNodeVisitor{
public:

    DEFINE_RIGIDBODY_CONFIG_TYPES

    typedef TContactGraph ContactGraphType;
    typedef typename ContactGraphType::NodeDataType NodeDataType;
    typedef typename ContactGraphType::EdgeDataType EdgeDataType;
    typedef typename ContactGraphType::EdgeType EdgeType;
    typedef typename ContactGraphType::NodeType NodeType;

    SorProxInitNodeVisitor()
    {}

    void setLog(Logging::Log * solverLog){
        m_pSolverLog = solverLog;
    }

    // Set Sor Prox parameter
    void setParams(PREC alpha){
        m_alpha = alpha;
    }

    void visitNode(NodeType & node){

        typename ContactGraphType::NodeDataType & nodeData = node.m_nodeData;

        // Get lambda from percussion pool otherwise set to zero
        // TODO
        nodeData.m_LambdaBack.setZero();

        // (1+e)*xi -> b
        nodeData.m_b = nodeData.m_I_plus_eps.asDiagonal() * nodeData.m_chi;

        // u_0 , calculate const b
        // First Body
        if(nodeData.m_pCollData->m_pBody1->m_eState == RigidBodyType::BodyState::SIMULATED) {



            // m_back contains u_s + M^⁻1*h*deltaT already!
            // add + initial values M^⁻1 W lambda0 from percussion pool
            nodeData.m_u1BufferPtr->m_front +=  nodeData.m_pCollData->m_pBody1->m_MassMatrixInv_diag.asDiagonal() * (nodeData.m_W_body1 * nodeData.m_LambdaBack );


            nodeData.m_b += nodeData.m_eps.asDiagonal() * nodeData.m_W_body1.transpose() * nodeData.m_u1BufferPtr->m_back /* m_u_s */ ;
            nodeData.m_G_ii += nodeData.m_W_body1.transpose() * nodeData.m_pCollData->m_pBody1->m_MassMatrixInv_diag.asDiagonal() * nodeData.m_W_body1 ;
        }
        // SECOND BODY!
        if(nodeData.m_pCollData->m_pBody2->m_eState == RigidBodyType::BodyState::SIMULATED ) {

            // m_back contains u_s + M^⁻1*h*deltaT already!
            nodeData.m_u2BufferPtr->m_front +=   nodeData.m_pCollData->m_pBody2->m_MassMatrixInv_diag.asDiagonal() * (nodeData.m_W_body2 * nodeData.m_LambdaBack ); /// + initial values M^⁻1 W lambda0 from percussion pool

            nodeData.m_b += nodeData.m_eps.asDiagonal() * nodeData.m_W_body2.transpose() *  nodeData.m_u1BufferPtr->m_back;
            nodeData.m_G_ii += nodeData.m_W_body2.transpose() * nodeData.m_pCollData->m_pBody2->m_MassMatrixInv_diag.asDiagonal() * nodeData.m_W_body2 ;
        }



        // Calculate R_ii
        nodeData.m_R_i_inv_diag(0) = m_alpha / (nodeData.m_G_ii(0,0));
        PREC r_T = m_alpha / ((nodeData.m_G_ii.diagonal().template tail<2>()).maxCoeff());
        nodeData.m_R_i_inv_diag(1) = r_T;
        nodeData.m_R_i_inv_diag(2) = r_T;

        #if CoutLevelSolverWhenContact>2
            LOG(m_pSolverLog, " nodeData.m_b :"<< nodeData.m_b.transpose() <<std::endl
                << " nodeData.m_G_ii :"<<std::endl<< nodeData.m_G_ii <<std::endl
                << " nodeData.m_R_i_inv_diag: "<< nodeData.m_R_i_inv_diag.transpose() <<std::endl;);
        #endif

        #if CoutLevelSolverWhenContact>2
            LOG(m_pSolverLog,  " nodeData.m_mu: "<< nodeData.m_mu <<std::endl;);
        #endif


    }

private:
    Logging::Log * m_pSolverLog;
    PREC m_alpha;
};

/**
@brief Visitor for class ContactGraph
*/
template<typename TContactGraph>
class SorProxInitSplitNodeVisitor{
public:
    DEFINE_LAYOUT_CONFIG_TYPES
    typedef TContactGraph ContactGraphType;

    typedef typename ContactGraphType::SplitBodyNodeDataType NodeType;

    SorProxInitSplitNodeVisitor()
    {}

    void setLog(Logging::Log * solverLog){
        m_pSolverLog = solverLog;
    }

    void visitNode(NodeType& node){
        auto mult = node.getMultiplicity();
        node.m_uBack.setZero(NDOFuObj*mult);
        node.m_uFront.setZero(NDOFuObj*mult);
        node.m_LambdaFront.setZero( NDOFuObj * node.m_nLambdas);
        node.m_gamma.setZero(node.m_nLambdas);
    }

private:
    Logging::Log * m_pSolverLog;
    const InclusionSolverSettings & m_Settings;
    bool & m_bConverged; ///< Access to global flag for cancelation criteria
    const unsigned int & m_iterationsNeeded; ///< Access to global iteration counter

};

/**
@brief Visitor for class ContactGraph
*/
template<typename TContactGraph>
class ComputeMultiplicitySplitNodeVisitor{
public:

    typedef TContactGraph ContactGraphType;

    typedef typename ContactGraphType::SplitBodyNodeDataType NodeType;

    ComputeMultiplicitySplitNodeVisitor(){};

    void visitNode(NodeType& node){
        auto mult = node.getMultiplicity();
        node.m_multiplicityWeights.setConstant(mult,1.0/mult);
    }

};

#endif // ContactGraphVisitorMPI_hpp
