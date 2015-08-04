#ifndef GRSF_Dynamics_Inclusion_ContactGraphVisitorsMPI_hpp
#define GRSF_Dynamics_Inclusion_ContactGraphVisitorsMPI_hpp

#include "GRSF/Common/TypeDefs.hpp"
#include "GRSF/Common/LogDefines.hpp"

#include "GRSF/Common/SimpleLogger.hpp"

#include "GRSF/Dynamics/Inclusion/ContactModels.hpp"
#include "GRSF/Dynamics/Inclusion/ProxFunctions.hpp"
#include InclusionSolverSettings_INCLUDE_FILE

#include "GRSF/Dynamics/Inclusion/ContactGraphVisitors.hpp"

/**
@brief Visitor for class ContactGraph
*/
template<typename TContactGraph>
class SorProxStepSplitNodeVisitor{
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

    using ContactGraphType = TContactGraph;


    using NodeDataType = typename ContactGraphType::SplitBodyNodeDataType;

    SorProxStepSplitNodeVisitor(const InclusionSolverSettingsType &settings, bool & globalConverged, const unsigned int & globalIterationNeeded):
                           m_settings(settings),m_bConverged(globalConverged),
                           m_globalIterationCounter(globalIterationNeeded)
    {}

    template<typename TNode>
    inline void operator()(TNode & node){
        dispatch(node.getData());
    }

    void setLog(Logging::Log * solverLog){
        m_pSolverLog = solverLog;
    }

    void dispatch(NodeDataType& node){

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
        for(unsigned int i = 0; i<mult; i++){
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
class SorProxInitSplitNodeVisitor{
public:
    DEFINE_LAYOUT_CONFIG_TYPES
    using ContactGraphType = TContactGraph;

    using NodeDataType = typename ContactGraphType::SplitBodyNodeDataType;

    template<typename TNode>
    void operator()(TNode & node){
        dispatch(node.getData());
    }

    void dispatch(NodeDataType& nodeData){
        nodeData.initData();
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

    using NodeDataType = typename ContactGraphType::SplitBodyNodeDataType;

    SplitNodeCheckUpdateVisitor()
    {}

    template<typename TNode>
    void operator()(TNode & node){
        dispatch(node.getData());
    }

    inline void dispatch(NodeDataType& node){
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


#include "GRSF/Dynamics/General/RigidBodyFunctionsMPI.hpp"


template<typename TContactGraph>
class SetWeightingLocalBodiesSplitNodeVisitor{
public:

    using ContactGraphType = TContactGraph;

    using NodeDataType = typename ContactGraphType::SplitBodyNodeDataType;

    SetWeightingLocalBodiesSplitNodeVisitor(){};

    template<typename TNode>
    inline void operator()(TNode & node){
        dispatch(node.getData());
    }

    inline void dispatch(NodeDataType& node){
        auto mult = node.getMultiplicity();
        RigidBodyFunctions::changeBodyToSplitWeighting(node.m_pBody, mult, node.m_multiplicityWeights(0));
    }
};

template<typename TContactGraph>
class ResetWeightingLocalBodiesSplitNodeVisitor{
public:

    using ContactGraphType = TContactGraph;

    using NodeDataType = typename ContactGraphType::SplitBodyNodeDataType;

    ResetWeightingLocalBodiesSplitNodeVisitor(){};

    template<typename TNode>
    inline void operator()(TNode & node){
        dispatch(node.getData());
    }

    inline void dispatch(NodeDataType& node){
        RigidBodyFunctions::changeBodyToNormalWeighting(node.m_pBody);
    }
};



#endif // ContactGraphVisitorMPI_hpp
