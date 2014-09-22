#ifndef ContactGraphVisitorMPI_hpp
#define ContactGraphVisitorMPI_hpp

#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include "SimpleLogger.hpp"

#include "ContactModels.hpp"
#include "ProxFunctions.hpp"
#include InclusionSolverSettings_INCLUDE_FILE



/**
@brief Visitor for class ContactGraph
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
                           bool & globalConverged, const unsigned int & globalIterationNeeded):
            m_settings(settings),m_bConverged(globalConverged),
            m_globalIterationCounter(globalIterationNeeded)
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

            LOGSLLEVEL3_CONTACT(m_pSolverLog, "---> SorProx, Normal Node: " << node.m_nodeNumber <<"====================="<<  std::endl);
             if( nodeData.m_pCollData->m_pBody1->m_eMode == RigidBodyType::BodyMode::SIMULATED  &&  nodeData.m_pCollData->m_pBody2->m_eMode == RigidBodyType::BodyMode::SIMULATED){
               LOGSLLEVEL3_CONTACT(m_pSolverLog, "---> Sim<->Sim Node:"<<  std::endl);
            }

        if( nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCF ) {

            using CMT = typename CONTACTMODELTYPE(ContactModels::Enum::UCF);
            // Init the prox value
            nodeData.m_LambdaFront = nodeData.m_b;
//                LOGSLLEVEL3_CONTACT(m_pSolverLog, "\t---> nd.b: " << nodeData.m_b.transpose() << std::endl);
//                LOGSLLEVEL3_CONTACT(m_pSolverLog, "\t---> nd.W1_T: " <<std::endl<< nodeData.m_W_body1.transpose() << std::endl);
//                LOGSLLEVEL3_CONTACT(m_pSolverLog, "\t---> nd.W2_T: " <<std::endl<< nodeData.m_W_body2.transpose() << std::endl);

            // FIRST BODY!
            if( nodeData.m_pCollData->m_pBody1->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {
                nodeData.m_LambdaFront += nodeData.m_W_body1.transpose() * nodeData.m_u1BufferPtr->m_front ;
            }
            // SECOND BODY!
            if( nodeData.m_pCollData->m_pBody2->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {
                nodeData.m_LambdaFront += nodeData.m_W_body2.transpose() * nodeData.m_u2BufferPtr->m_front;
            }


//          LOGSLLEVEL3_CONTACT(m_pSolverLog, "\t---> nd.chi: " << nodeData.m_LambdaFront.transpose() << std::endl);

            nodeData.m_LambdaFront = -(nodeData.m_R_i_inv_diag.asDiagonal() * nodeData.m_LambdaFront).eval(); //No alias due to diagonal!!! (if normal matrix multiplication there is aliasing!
            nodeData.m_LambdaFront += nodeData.m_LambdaBack;
            //Prox

            Prox::ProxFunction<ConvexSets::RPlusAndDisk>::doProxSingle(
                nodeData.m_contactParameter.m_params[CMT::muIdx], nodeData.m_LambdaFront);

            LOGSLLEVEL3_CONTACT(m_pSolverLog, "\t---> nd.m_LambdaBack: "  << nodeData.m_LambdaBack.transpose() << std::endl);
            LOGSLLEVEL3_CONTACT(m_pSolverLog, "\t---> nd.m_LambdaFront: " << nodeData.m_LambdaFront.transpose() << std::endl);
            if(Numerics::cancelCriteriaValue(nodeData.m_LambdaBack,nodeData.m_LambdaFront,m_settings.m_AbsTol, m_settings.m_RelTol)){
              LOGSLLEVEL3_CONTACT(m_pSolverLog, "\t---> Lambda converged"<<std::endl);
            }


            // u_k+1 = u_k + M^-1 W (lambda_k+1 - lambda_k)
            // FIRST BODY!
            if( nodeData.m_pCollData->m_pBody1->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {
//                    LOGSLLEVEL3_CONTACT(m_pSolverLog, "\t---> body1.massInv: " << nodeData.m_pCollData->m_pBody1->m_MassMatrixInv_diag.transpose() << std::endl;)
//                    LOGSLLEVEL3_CONTACT(m_pSolverLog, "\t---> body1.h_term: " << nodeData.m_pCollData->m_pBody1->m_h_term.transpose() << std::endl;)
                uCache1 = nodeData.m_u1BufferPtr->m_front;
                nodeData.m_u1BufferPtr->m_front = nodeData.m_u1BufferPtr->m_front
                                                  + nodeData.m_pCollData->m_pBody1->m_MassMatrixInv_diag.asDiagonal()
                                                  * nodeData.m_W_body1 * ( nodeData.m_LambdaFront - nodeData.m_LambdaBack );

                LOGSLLEVEL3_CONTACT(m_pSolverLog,"\t---> nd.u1Front: " << nodeData.m_u1BufferPtr->m_front.transpose() << std::endl);


//                if(m_settings.m_eConvergenceMethod == InclusionSolverSettingsType::InVelocityLocal) {
//                    if(m_globalIterationCounter >= m_settings.m_MinIter && m_bConverged) {
//                        nodeData.m_bConverged  = Numerics::cancelCriteriaValue(uCache1,nodeData.m_u1BufferPtr->m_front,m_settings.m_AbsTol, m_settings.m_RelTol);
//                        if(!nodeData.m_bConverged ) {
//                            //converged stays false;
//                            // Set global Converged = false;
//                            m_bConverged = false;
//                            *m_pSolverLog << "m_bConverged = false;"<<std::endl;
//                        }
//                    } else {
//                        m_bConverged=false;
//                    }
//                }else if(m_settings.m_eConvergenceMethod == InclusionSolverSettingsType::InEnergyLocalMix){
//                    if(m_globalIterationCounter >= m_settings.m_MinIter && m_bConverged) {
//                        nodeData.m_bConverged  = Numerics::cancelCriteriaMatrixNormSq(   uCache1,
//                                                                          nodeData.m_pCollData->m_pBody1->m_MassMatrix_diag,
//                                                                          nodeData.m_LambdaBack,
//                                                                          nodeData.m_LambdaFront,
//                                                                          nodeData.m_G_ii,
//                                                                          m_settings.m_AbsTol,
//                                                                          m_settings.m_RelTol);
//                        if(!nodeData.m_bConverged ) {
//                            //converged stays false;
//                            // Set global Converged = false;
//                            m_bConverged = false;
//                        }
//                    } else {
//                        m_bConverged=false;
//                    }
//                }

            }
            // SECOND BODY
            if( nodeData.m_pCollData->m_pBody2->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {

//                    LOGSLLEVEL3_CONTACT(m_pSolverLog, "\t---> body1.massInv: " << nodeData.m_pCollData->m_pBody2->m_MassMatrixInv_diag.transpose() << std::endl;)
//                    LOGSLLEVEL3_CONTACT(m_pSolverLog, "\t---> body1.h_term: " << nodeData.m_pCollData->m_pBody2->m_h_term.transpose() << std::endl;)

                uCache2 = nodeData.m_u2BufferPtr->m_front;
                nodeData.m_u2BufferPtr->m_front = nodeData.m_u2BufferPtr->m_front
                                                    + nodeData.m_pCollData->m_pBody2->m_MassMatrixInv_diag.asDiagonal()
                                                    * nodeData.m_W_body2 * ( nodeData.m_LambdaFront - nodeData.m_LambdaBack );

                LOGSLLEVEL3_CONTACT(m_pSolverLog,"\t---> nd.u2Front: " << nodeData.m_u2BufferPtr->m_front.transpose() << std::endl);

//                if(m_settings.m_eConvergenceMethod == InclusionSolverSettingsType::InVelocityLocal) {
//                    if(m_globalIterationCounter >= m_settings.m_MinIter && m_bConverged) {
//                        nodeData.m_bConverged  = Numerics::cancelCriteriaValue(uCache2,
//                                                                  nodeData.m_u2BufferPtr->m_front,
//                                                                  m_settings.m_AbsTol,
//                                                                  m_settings.m_RelTol);
//                        if(!nodeData.m_bConverged ) {
//                            //converged stays false;
//                            // Set global Converged = false;
//                            m_bConverged = false;
//                        }
//                    } else {
//                        m_bConverged=false;
//                    }
//
//                }else if(m_settings.m_eConvergenceMethod == InclusionSolverSettingsType::InEnergyLocalMix){
//                    if(m_globalIterationCounter >= m_settings.m_MinIter && m_bConverged) {
//                        nodeData.m_bConverged  = Numerics::cancelCriteriaMatrixNormSq(   uCache2,
//                                                                          nodeData.m_pCollData->m_pBody2->m_MassMatrix_diag,
//                                                                          nodeData.m_LambdaBack,
//                                                                          nodeData.m_LambdaFront,
//                                                                          nodeData.m_G_ii,
//                                                                          m_settings.m_AbsTol,
//                                                                          m_settings.m_RelTol);
//                        if(!nodeData.m_bConverged ) {
//                            //converged stays false;
//                            // Set global Converged = false;
//                            m_bConverged = false;
//                        }
//                    } else {
//                        m_bConverged=false;
//                    }
//                }
            }

//            if(m_settings.m_eConvergenceMethod == InclusionSolverSettingsType::InLambda) {
//                if(m_globalIterationCounter >= m_settings.m_MinIter && m_bConverged) {
//                    nodeData.m_bConverged = Numerics::cancelCriteriaValue(nodeData.m_LambdaBack,nodeData.m_LambdaFront,m_settings.m_AbsTol, m_settings.m_RelTol);
//                    if(!nodeData.m_bConverged) {
//                        //converged stays false;
//                        // Set global Converged = false;
//                        m_bConverged = false;
//                    }
//                } else {
//                    m_bConverged=false;
//                }
//            }

            // Swap Lambdas, but dont swap Velocities...
            // Swap velocities when we finished ONE Sor Prox Iteration! (very important!)
            nodeData.swapLambdas();


        } else {
            ASSERTMSG(false," You specified a contact model which has not been implemented so far!");
        }
    }

private:
    Logging::Log * m_pSolverLog;
    const InclusionSolverSettingsType & m_settings;
    bool & m_bConverged; ///< Access to global flag for cancelation criteria
    const unsigned int & m_globalIterationCounter; ///< Access to global iteration counter

};


/**
@brief Visitor for class ContactGraph
*/
template<typename TContactGraph>
class SorProxStepSplitNodeVisitor{
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

    using ContactGraphType = TContactGraph;


    using NodeType = typename ContactGraphType::SplitBodyNodeDataType;

    SorProxStepSplitNodeVisitor(const InclusionSolverSettingsType &settings, bool & globalConverged, const unsigned int & globalIterationNeeded):
                           m_settings(settings),m_bConverged(globalConverged),
                           m_globalIterationCounter(globalIterationNeeded)
    {}

    void setLog(Logging::Log * solverLog){
        m_pSolverLog = solverLog;
    }

    void visitNode(NodeType& node){

        // Calculate the exact values for the billateral split nodes

        LOGSLLEVEL3_CONTACT(m_pSolverLog, "---> SorProx, Billateral Node: ====================="<<  std::endl
                << "\t---> local body id: " << RigidBodyId::getBodyIdString(node.m_pBody->m_id) << std::endl);

        auto mult = node.getMultiplicity();

            LOGSLLEVEL3_CONTACT(m_pSolverLog,"\t---> multiplicity: " << mult << std::endl;)


        // Copy local velocity
        node.m_uBack.template head<NDOFuBody>() = node.m_pBody->m_pSolverData->m_uBuffer.m_front;

            LOGSLLEVEL3_CONTACT(m_pSolverLog,"\t---> uBack: " << node.m_uBack.transpose() <<std::endl;);

        // Build gamma = [u0-u1, u1-u2, u2-u3] for multiplicity = 4
        node.m_gamma =   node.m_uBack.head(NDOFuBody*node.m_nConstraints) -
                           node.m_uBack.segment(NDOFuBody, NDOFuBody*node.m_nConstraints);

            LOGSLLEVEL3_CONTACT(m_pSolverLog, "\t---> nd.m_gamma: " << node.m_gamma.transpose() << std::endl;);

        // calculate L⁻¹*gamma = Lambda, where L⁻¹ is the matrix choosen by the multiplicity
            //LOGSLLEVEL3_CONTACT(m_pSolverLog, "\t---> nd.LInv: " << std::endl;);
            //LOGSLLEVEL3_CONTACT(m_pSolverLog, node.getLInvMatrix() << std::endl;);

        node.m_deltaLambda = node.getLInvMatrix() *-1*node.m_gamma;

            LOGSLLEVEL3_CONTACT(m_pSolverLog, "\t---> nd.m_deltaLambda: " << node.m_deltaLambda.transpose() << std::endl;);

        //Propagate billateral forces to velocities:
        // The sign is contained in the m_multiplicityWeights vector
        // u_G_End = uBack + M_G⁻¹ * W_M * deltaLambda_M

        node.m_uFront.setZero();
        node.m_uFront.segment(0,NDOFuBody*node.m_nConstraints) = node.m_deltaLambda;
        node.m_uFront.template segment(NDOFuBody,NDOFuBody*node.m_nConstraints) -= node.m_deltaLambda;
        for(int i = 0; i<mult; i++){
            node.m_uFront.segment(NDOFuBody*i,NDOFuBody) *= 1.0 / node.m_multiplicityWeights(i);
        }
        node.m_uFront  +=  node.m_uBack;

//            LOGSLLEVEL3_CONTACT(m_pSolverLog, "\t---> nd.m_uFront: " << node.m_uFront.transpose() <<std::endl; );

        // Because we solve this billateral contact directly, we are converged for this node!
        // no change of the flag m_bConverged

        //Copy local back
        node.m_pBody->m_pSolverData->m_uBuffer.m_front = node.m_uFront.template head<NDOFuBody>();
    }

private:
    Logging::Log * m_pSolverLog;
    const InclusionSolverSettingsType & m_settings;
    bool & m_bConverged; ///< Access to global flag for cancelation criteria
    const unsigned int & m_globalIterationCounter; ///< Access to global iteration counter

};




/**
@brief Visitor for class ContactGraph
*/
template<typename TContactGraph>
class SorProxInitNodeVisitor{
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

    using ContactGraphType = TContactGraph;
    using NodeDataType = typename ContactGraphType::NodeDataType;
    using EdgeDataType = typename ContactGraphType::EdgeDataType;
    using EdgeType = typename ContactGraphType::EdgeType;
    using NodeType = typename ContactGraphType::NodeType;

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

        LOGSLLEVEL3_CONTACT(m_pSolverLog, "---> SorProx, Init Node: " << node.m_nodeNumber <<"====================="<<  std::endl);

        typename ContactGraphType::NodeDataType & nodeData = node.m_nodeData;

        // Get lambda from percussion pool otherwise set to zero
        // TODO
        nodeData.m_LambdaBack.setZero();

        // (1+e)*xi -> b
        nodeData.m_b = (nodeData.m_eps.array() + 1).matrix().asDiagonal() * nodeData.m_chi;

        // u_0 , calculate const b
        // First Body
        if(nodeData.m_pCollData->m_pBody1->m_eMode == RigidBodyType::BodyMode::SIMULATED) {
            // m_back contains u_s + M^⁻1*h*deltaT already!
            // add + initial values M^⁻1 W lambda0 from percussion pool
            nodeData.m_u1BufferPtr->m_front +=  nodeData.m_pCollData->m_pBody1->m_MassMatrixInv_diag.asDiagonal() * (nodeData.m_W_body1 * nodeData.m_LambdaBack );


            nodeData.m_b += nodeData.m_eps.asDiagonal() * nodeData.m_W_body1.transpose() * nodeData.m_u1BufferPtr->m_back /* m_u_s */ ;
            nodeData.m_G_ii += nodeData.m_W_body1.transpose() * nodeData.m_pCollData->m_pBody1->m_MassMatrixInv_diag.asDiagonal() * nodeData.m_W_body1 ;
        }
        // SECOND BODY!
        if(nodeData.m_pCollData->m_pBody2->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {

            // m_back contains u_s + M^⁻1*h*deltaT already!
            nodeData.m_u2BufferPtr->m_front +=   nodeData.m_pCollData->m_pBody2->m_MassMatrixInv_diag.asDiagonal() * (nodeData.m_W_body2 * nodeData.m_LambdaBack ); /// + initial values M^⁻1 W lambda0 from percussion pool

            nodeData.m_b += nodeData.m_eps.asDiagonal() * nodeData.m_W_body2.transpose() *  nodeData.m_u2BufferPtr->m_back;
            nodeData.m_G_ii += nodeData.m_W_body2.transpose() * nodeData.m_pCollData->m_pBody2->m_MassMatrixInv_diag.asDiagonal() * nodeData.m_W_body2 ;
        }

//            LOGSLLEVEL3_CONTACT(m_pSolverLog,  " nodeData.m_eps: "<< nodeData.m_eps.transpose() <<std::endl;);
//            LOGSLLEVEL3_CONTACT(m_pSolverLog,  " nodeData.m_b: "<< nodeData.m_b.transpose() <<std::endl;);

        // Calculate R_ii
        nodeData.m_R_i_inv_diag(0) = m_alpha / (nodeData.m_G_ii(0,0));
        PREC r_T = m_alpha / ((nodeData.m_G_ii.diagonal().template tail<2>()).maxCoeff());
        nodeData.m_R_i_inv_diag(1) = r_T;
        nodeData.m_R_i_inv_diag(2) = r_T;

//            LOGSLLEVEL3_CONTACT(m_pSolverLog, " nodeData.m_b :"<< nodeData.m_b.transpose() <<std::endl
//                << " nodeData.m_G_ii :"<<std::endl<< nodeData.m_G_ii <<std::endl
//                << " nodeData.m_R_i_inv_diag: "<< nodeData.m_R_i_inv_diag.transpose() <<std::endl;);
//
//            LOGSLLEVEL3_CONTACT(m_pSolverLog,  " nodeData.m_mu: "<< nodeData.m_mu <<std::endl;);


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
    using ContactGraphType = TContactGraph;

    using NodeType = typename ContactGraphType::SplitBodyNodeDataType;

    SorProxInitSplitNodeVisitor()
    {}

    void visitNode(NodeType& node){

        auto mult = node.getMultiplicity();
        node.m_multiplicityWeights.setConstant(mult,1.0/mult);

        node.m_uBack.setZero(NDOFuBody*mult);
        node.m_uFront.setZero(NDOFuBody*mult);
        node.m_deltaLambda.setZero( NDOFuBody * node.m_nConstraints);
        node.m_gamma.setZero(node.m_nConstraints*NDOFuBody);
    }

};


/**
@brief Visitor for class ContactGraph
*/
template<typename TContactGraph>
class SplitNodeCheckUpdateVisitor{
public:
    DEFINE_LAYOUT_CONFIG_TYPES
    using ContactGraphType = TContactGraph;

    using NodeType = typename ContactGraphType::SplitBodyNodeDataType;

    SplitNodeCheckUpdateVisitor()
    {}

    void visitNode(NodeType& node){
        for( auto it = node.m_partRanks.begin(); it != node.m_partRanks.end(); ++it){
            if(it->second.m_bGotUpdate == false){
                ERRORMSG("Rank: " << it->first << " in SplitNode for body id: "
                         << RigidBodyId::getBodyIdString(node.m_pBody) << " has not got update!" );
            }else{
                it->second.m_bGotUpdate == false;
            }
        }
    }

};


#include "RigidBodyFunctionsMPI.hpp"


template<typename TContactGraph>
class SetWeightingLocalBodiesSplitNodeVisitor{
public:

    using ContactGraphType = TContactGraph;

    using NodeType = typename ContactGraphType::SplitBodyNodeDataType;

    SetWeightingLocalBodiesSplitNodeVisitor(){};


    inline void visitNode(NodeType& node){
        auto mult = node.getMultiplicity();
        RigidBodyFunctions::changeBodyToSplitWeighting(node.m_pBody, mult, node.m_multiplicityWeights(0));
    }
};

template<typename TContactGraph>
class ResetWeightingLocalBodiesSplitNodeVisitor{
public:

    using ContactGraphType = TContactGraph;

    using NodeType = typename ContactGraphType::SplitBodyNodeDataType;

    ResetWeightingLocalBodiesSplitNodeVisitor(){};

    inline void visitNode(NodeType& node){
        RigidBodyFunctions::changeBodyToNormalWeighting(node.m_pBody);
    }
};



#endif // ContactGraphVisitorMPI_hpp
