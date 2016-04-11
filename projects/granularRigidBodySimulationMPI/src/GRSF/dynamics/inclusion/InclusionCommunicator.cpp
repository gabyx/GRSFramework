// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include "InclusionCommunicator<Combo>.hpp"

#include "GRSF/dynamics/inclusion/ContactGraphMPI.hpp"

#include "GRSF/dynamics/inclusion/ContactGraphVisitorsMPI.hpp"


template<typename Combo>
InclusionCommunicator<Combo>::InclusionCommunicator<Combo>(std::shared_ptr< BodyCommunicator> pBodyComm,
        std::shared_ptr< DynamicsSystemType> pDynSys ,
        std::shared_ptr< ProcessCommunicatorType > pProcCom):

    m_pDynSys(pDynSys),
    m_globalLocal(pDynSys->m_SimBodies),
    m_globalRemote(pDynSys->m_RemoteSimBodies),
    m_pProcCom(pProcCom),
    m_pProcInfo(m_pProcCom->getProcInfo()),
    m_pProcTopo(m_pProcCom->getProcInfo()->getProcTopo()),
    m_pBodyComm(pBodyComm),
    m_nbDataMap(m_pProcCom->getProcInfo()->getRank()),
    m_rank(m_pProcCom->getProcInfo()->getRank()),
    m_nbRanks(m_pProcCom->getProcInfo()->getProcTopo()->getNeighbourRanks()),
    m_messageContact(this),m_messageMultiplicity(this) {
    if(Logging::LogManager::getSingletonPtr()->existsLog("SimulationLog")) {
        m_pSimulationLog = Logging::LogManager::getSingletonPtr()->getLog("SimulationLog");
    } else {
        GRSF_ERRORMSG("SimulationLog does not yet exist? Did you create it?")
    }

    // Initialize all NeighbourDatas
    for(auto rankIt = m_nbRanks.begin() ; rankIt != m_nbRanks.end(); rankIt++) {
        LOGIC(m_pSimulationLog,"--->InclusionCommunicator<Combo>: Add neighbour data for process rank: "<<*rankIt<<std::endl;);
        auto res = m_nbDataMap.insert(*rankIt);
        GRSF_ASSERTMSG(res.second,"Could not insert in m_nbDataMap for rank: " << *rankIt);
    }
    m_pSimulationLog->logMessage("--->InclusionCommunicator<Combo>: Initialized all NeighbourDatas");

//        m_nbRanksSending.reserve(m_nbRanks.size());
//        m_nbRanksReceiving.reserve(m_nbRanks.size());
//        m_nbRanksSending.append

    m_pSimulationLog->logMessage("---> Initialized InclusionCommunicator<Combo>");
}
template<typename Combo>
void InclusionCommunicator<Combo>::communicateRemoteContacts() {
    LOGIC(m_pSimulationLog,"---> InclusionCommunication: Send remote contacts (initialize global prox)"<< std::endl;)

    m_nbRanksSendRecvLocal.clear();
    m_nbRanksSendRecvRemote.clear();

    // First for each neighbour, communicate the ids of all remote body which have contacts
    sendContactMessageToNeighbours();

    receiveContactMessagesFromNeighbours();

    computeMultiplicityWeights();

    // Send all multiplicity factors of all local splitBodyNodes
    sendBodyMultiplicityMessageToNeighbours();
    // Recv all multiplicity factor of all remote bodies
    recvBodyMultiplicityMessageFromNeighbours();

    LOGIC(m_pSimulationLog,"---> InclusionCommunication: finished"<< std::endl;)
}

template<typename Combo>
void InclusionCommunicator<Combo>::sendContactMessageToNeighbours() {
    LOGIC(m_pSimulationLog,"MPI>\t Send message (EXTERNALCONTACTS_MESSAGE) to neighbours!"<<std::endl;)

    for(auto it = m_nbRanks.begin(); it != m_nbRanks.end(); it++) {
        LOGBC(m_pSimulationLog,"--->\t\t Send contact message to neighbours with rank: "<< *it <<std::endl;)
        // Instanciate a MessageWrapper which contains a boost::serialization function!
        m_messageContact.setRank(*it);
        m_pProcCom->sendMessageToRank(m_messageContact,*it, MPILayer::MPIMessageTag::Type::EXTERNALCONTACTS_MESSAGE );
    }
    LOGBC(m_pSimulationLog,"MPI>\t Send finished!"<<std::endl;)
}
template<typename Combo>
void InclusionCommunicator<Combo>::receiveContactMessagesFromNeighbours() {
    LOGIC(m_pSimulationLog,"MPI>\t Receive all messages (EXTERNALCONTACTS_MESSAGE) from neighbours!"<<std::endl;)
    // set the rank of the receiving message automatically! inside the function!
    m_pProcCom->receiveMessageFromRanks(m_messageContact, m_nbRanks, MPILayer::MPIMessageTag::Type::EXTERNALCONTACTS_MESSAGE );
    LOGIC(m_pSimulationLog,"MPI>\t Receive finished!"<<std::endl;)

    // Wait for all sends to complete, Important because we issue a nonblocking send in sendMessagesToNeighbours
    m_pProcCom->waitForAllSends();
}
template<typename Combo>
void InclusionCommunicator<Combo>::sendBodyMultiplicityMessageToNeighbours() {
    LOGIC(m_pSimulationLog,"MPI>\t Send message (SPLITBODYFACTOR_MESSAGE) to neighbours!"<<std::endl;)

    for(auto it = m_nbRanksSendRecvLocal.begin(); it != m_nbRanksSendRecvLocal.end(); it++) {
        LOGBC(m_pSimulationLog,"--->\t\t Send contact message to neighbours with rank: "<< *it <<std::endl;)
        // Instanciate a MessageWrapper which contains a boost::serialization function!
        m_messageMultiplicity.setRank(*it);
        m_pProcCom->sendMessageToRank(m_messageMultiplicity,*it, MPILayer::MPIMessageTag::Type::SPLITBODYFACTOR_MESSAGE );
    }
    LOGBC(m_pSimulationLog,"MPI>\t Send finished!"<<std::endl;)


}
template<typename Combo>
void InclusionCommunicator<Combo>::recvBodyMultiplicityMessageFromNeighbours() {
    LOGIC(m_pSimulationLog,"MPI>\t Receive all messages (SPLITBODYFACTOR_MESSAGE) from neighbours!"<<std::endl;)
    // set the rank of the receiving message automatically! inside the function!
    // m_nbRanksSendRecvRem#ote.erase(m_nbRanksSendRecvRemote.begin()); with this in, it obviously hangs!!
    // so the code is correct! all messages get received
    m_pProcCom->receiveMessageFromRanks(m_messageMultiplicity, m_nbRanksSendRecvRemote, MPILayer::MPIMessageTag::Type::SPLITBODYFACTOR_MESSAGE );
    LOGIC(m_pSimulationLog,"MPI>\t Receive finished!"<<std::endl;)

    // Wait for all sends to complete, Important because we issue a nonblocking send in sendMessagesToNeighbours
    // We dont send to all neighbours but this should not be a problem for the underlying MPI_Wait call
    m_pProcCom->waitForAllSends();
}
template<typename Combo>
void InclusionCommunicator<Combo>::computeMultiplicityWeights(){

    ComputeMultiplicitySplitNodeVisitor<ContactGraphType> v;
    m_pContactGraph->applyNodeVisitorSplitBody(v);

}

