// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include "GRSF/dynamics/general/BodyCommunicator.hpp"

#include "GRSF/dynamics/general/MPIMessageTag.hpp"

BodyCommunicator::BodyCommunicator(std::shared_ptr<DynamicsSystemType> pDynSys,
                                   std::shared_ptr<ProcessCommunicatorType> pProcComm)
    : m_pDynSys(pDynSys)
    , m_pProcComm(pProcComm)
    , m_rank(m_pProcComm->getRank())
    , m_globalLocal(pDynSys->m_simBodies)
    , m_globalRemote(pDynSys->m_remoteSimBodies)
    , m_globalGeometries(pDynSys->m_globalGeometries)
    , m_nbDataMap(m_pProcComm->getRank())
    , m_message(this)
{
    if (Logging::LogManager::getSingleton().existsLog("SimulationLog"))
    {
        m_pSimulationLog = Logging::LogManager::getSingleton().getLog("SimulationLog");
    }
    else
    {
        GRSF_ERRORMSG("SimulationLog does not yet exist? Did you create it?")
    }
}

void BodyCommunicator::reset()
{
    resetTopology();
}

void BodyCommunicator::resetTopology()
{
    m_pProcTopo = m_pProcComm->getProcTopo();
    m_nbRanks   = m_pProcComm->getProcTopo()->getNeighbourRanks();

    // Clear Neighbour Map
    m_nbDataMap.clear();

    // Initialize all NeighbourDatas
    for (auto& rank : m_nbRanks)
    {
        LOGBC(m_pSimulationLog, "---> BodyCommunicator: Add neighbour data for process rank: " << rank << std::endl;);
        auto res = m_nbDataMap.insert(rank);
        GRSF_ASSERTMSG(res.second, "Could not insert in m_nbDataMap for rank: " << rank);
    }
    m_pSimulationLog->logMessage("---> BodyCommunicator: Initialized all NeighbourDatas");

    // Fill in all BodyInfos for the local bodies (remote bodies are not considered, there should not be any of those)
    for (auto* body : m_globalLocal)
    {
        GRSF_ASSERTMSG(m_pProcTopo->belongsBodyToProcess(body),
                       "Body with id: " << RigidBodyId::getBodyIdString(body)
                                        << " does not belong to process? How did you initialize your bodies?")
        if (body->m_pBodyInfo)
        {
            delete body->m_pBodyInfo;
        }
        body->m_pBodyInfo = new RigidBodyType::BodyInfoType(m_rank);
    }

    m_pSimulationLog->logMessage("---> Initialized BodyCommunicator");
}

void BodyCommunicator::communicate(PREC currentSimTime)
{
    LOGBC(m_pSimulationLog,
          "---> BodyCommunication: Send and Receive message from/to neighbours, t = " << currentSimTime << std::endl;)

    m_currentSimTime = currentSimTime;

    // Reset all neighbour ranks which are empty!
    m_nbRanksEmpty.clear();

    // Find all local bodies which overlap
    typename ProcessTopologyType::NeighbourRanksListType neighbours;

    LOGBC(m_pSimulationLog, "--->\t Update neighbour data structures with LOCAL bodies:" << std::endl;)
    for (auto* body : m_globalLocal)
    {
        // LOGBC(m_pSimulationLog,"--->\t\t Reset neighbours flags..."<<std::endl;)
        body->m_pBodyInfo
            ->resetNeighbourFlags();  // Reset the overlap flag to false! (addLocalBodyExclusive uses this assumption!)

        // Check overlapping processes
        bool overlapsOwnProcess;
        // LOGBC(m_pSimulationLog,"--->\t\t Overlap Test..."<<std::endl;)
        bool overlapsNeighbours = m_pProcTopo->checkOverlap(body, neighbours, overlapsOwnProcess);

        LOGASSERTMSG(
            !(overlapsNeighbours == false && overlapsOwnProcess == false),
            m_pSimulationLog,
            "Body with id: " << RigidBodyId::getBodyIdString(body) << " does not belong to our process rank: " << m_rank
                             << " nor to any neighbour!")

        // Insert this body into the underlying structure for all nieghbours exclusively! (if no overlap, it is removed
        // everywhere)
        // LOGBC(m_pSimulationLog,"--->\t\t Add neighbours exclusively..."<<std::endl;)
        addLocalBodyExclusiveToNeighbourMap(body, neighbours);
        //        m_nbDataMap.addLocalBodyExclusive(body,neighbours);

        // Check owner of this body
        RankIdType ownerRank;
        m_pProcTopo->belongsBodyToProcess(body, ownerRank);
        // Check if belonging rank is in the neighbours or our own
        if (ownerRank != m_rank)
        {
            if (m_nbRanks.find(ownerRank) == m_nbRanks.end())
            {
                GRSF_ERRORMSG("---> Body with id: " << RigidBodyId::getBodyIdString(body)
                                                    << " belongs to no neighbour, ownerRank: "
                                                    << ownerRank
                                                    << " pos: "
                                                    << body->m_r_S);
            }
            LOGBC(m_pSimulationLog,
                  "--->\t\t Body with id: " << RigidBodyId::getBodyIdString(body) << " has owner rank: " << (ownerRank)
                                            << ", proccess rank: "
                                            << m_pProcComm->getRank()
                                            << std::endl;)
        }

        // Set the owning rank for this body:

        body->m_pBodyInfo->m_ownerRank        = ownerRank;
        body->m_pBodyInfo->m_overlapsThisRank = overlapsOwnProcess;

        // Status output
        //        typename ProcessTopologyType::NeighbourRanksListType::iterator itRank;
        //        for(itRank = neighbours.begin(); itRank != neighbours.end(); itRank++) {
        //            LOGBC(m_pSimulationLog,"--->\t\t Body with id: " << RigidBodyId::getBodyIdString(body) <<"
        //            overlaps Neigbour with Rank: "<< (*itRank) <<std::endl;)
        //        }
    }
    LOGBC(m_pSimulationLog, "--->\t Update neighbour data structures complete!" << std::endl;)

    // printAllNeighbourRanks();
    sendMessagesToNeighbours();
    cleanUp();

    receiveMessagesFromNeighbours();

    if (checkReceiveForRemotes())
    {
        LOGBC(m_pSimulationLog, "--->\t Update for Remotes: OK" << std::endl;)
    }
    else
    {
        LOGASSERTMSG(false, m_pSimulationLog, "--->\t Update for Remotes: FAILED" << std::endl)
    }

    LOGBC(m_pSimulationLog, "---> BodyCommunication: finished" << std::endl;)
}

template <typename List>
void BodyCommunicator::addLocalBodyExclusiveToNeighbourMap(RigidBodyType* body, const List& neighbourRanks)
{
    GRSF_STATIC_ASSERT((std::is_same<RankIdType, typename List::value_type>::value));
    // Add this local body exclusively to the given neighbours

    // Loop over all incoming  ranks
    for (auto& rank : neighbourRanks)
    {
        LOGBC(m_pSimulationLog,
              "Body id: " << RigidBodyId::getBodyIdString(body) << " overlaps rank: " << rank << std::endl;);

        // insert the new element into body info --> (rank, flags)
        std::pair<typename BodyProcessInfo::RankToFlagsType::iterator, bool> res =
            body->m_pBodyInfo->m_neighbourRanks.emplace(rank, typename RigidBodyType::BodyInfoType::Flags(true));

        res.first->second.m_overlaps = true;  // set the Flags for the existing or the newly inserted entry (rank,flags)

        if (res.second)
        {  // if inserted we need to add this body to the underlying neighbour data
            // add to the data
            auto* nbData = m_nbDataMap.getNeighbourData(rank);
            GRSF_ASSERTMSG(nbData, "No neighbour data for rank" << rank)

            auto pairlocalData = nbData->addLocalBodyData(body);
            GRSF_ASSERTMSG(pairlocalData.second,
                           "Insert to neighbour data rank: " << rank << " in process rank: " << m_rank << " failed!");
            pairlocalData.first->m_commStatus =
                NeighbourMapType::DataType::LocalDataType::SEND_NOTIFICATION;  // No need because is set automatically
                                                                               // in constructor
        }
        else
        {
            GRSF_ASSERTMSG(m_nbDataMap.getNeighbourData(rank), "No neighbour data for rank " << rank)
            GRSF_ASSERTMSG(
                m_nbDataMap.getNeighbourData(rank)->getLocalBodyData(body),
                "body with id " << RigidBodyId::getBodyIdString(body) << " in neighbour structure rank: " << rank
                                << " does not exist?");
            GRSF_ASSERTMSG(m_nbDataMap.getNeighbourData(rank)->getLocalBodyData(body)->m_commStatus ==
                               NeighbourMapType::DataType::LocalDataType::SEND_UPDATE,
                           "m_commStatus for body with id: " << RigidBodyId::getBodyIdString(body)
                                                             << " in neighbour structure rank: "
                                                             << rank
                                                             << "should be in update mode!");
        }
    }

    // Clean up of the neighbour structure is done after communication!
    // We cannot already remove the local bodies from the neighbours for
    // which the flag m_overlaps = false because, this needs to be communicated first! (such that any neighbour does not
    // request any update anymore!)
    // So set the flags for this body to SEND_REMOVE for all his neighbours which have m_overlaps = false

    for (typename BodyProcessInfo::RankToFlagsType::value_type& rankToFlags : body->m_pBodyInfo->m_neighbourRanks)
    {
        if (rankToFlags.second.m_overlaps == false)
        {
            auto* nbData = m_nbDataMap.getNeighbourData(rankToFlags.first);
            GRSF_ASSERTMSG(nbData, "No neighbour data for rank " << rankToFlags.first)

            auto* localData = nbData->getLocalBodyData(body);

            if (localData->m_commStatus == NeighbourMapType::DataType::LocalDataType::SEND_UPDATE)
            {
                localData->m_commStatus = NeighbourMapType::DataType::LocalDataType::SEND_REMOVE;
            }
            else if (localData->m_commStatus == NeighbourMapType::DataType::LocalDataType::SEND_NOTIFICATION)
            {
                // Falls notification
            }
        }
    }
}

bool BodyCommunicator::checkReceiveForRemotes()
{
    bool m_ok = true;

    for (auto* body : m_globalRemote)
    {
        LOGASSERTMSG(body->m_pBodyInfo, m_pSimulationLog, "bodyInfoPtr is nullptr! ");

        if (body->m_pBodyInfo->m_receivedUpdate == false)
        {
            LOGBC(m_pSimulationLog,
                  "---> WARNING: Remote body with id: " << RigidBodyId::getBodyIdString(body)
                                                        << " has not received an update!"
                                                        << std::endl;)
            m_ok = false;
        }
        else
        {
            // Set to false for next iteration!
            body->m_pBodyInfo->m_receivedUpdate = false;
        }
    }
    return m_ok;
}

void BodyCommunicator::sendMessagesToNeighbours()
{
    LOGBC(m_pSimulationLog, "MPI>\t Send messages (BODY_MESSAGE) to neighbours!" << std::endl;)
    m_localBodiesToDelete.clear();

    // Set flags
    m_message.setFlags(m_pDynSys->m_simBodies.size() == 0);

    for (typename ProcessTopologyType::NeighbourRanksListType::const_iterator it = m_nbRanks.begin();
         it != m_nbRanks.end();
         it++)
    {
        LOGBC(m_pSimulationLog, "--->\t\t Send message to neighbours with rank: " << *it << std::endl;)
        // Instanciate a MessageWrapper which contains a boost::serialization function!
        m_message.setRank(*it);
        m_pProcComm->sendMessageToNeighbourRank_async(m_message, *it, m_message.m_tag);
    }
    LOGBC(m_pSimulationLog, "MPI>\t Send finished!" << std::endl;)
}

void BodyCommunicator::receiveMessagesFromNeighbours()
{
    LOGBC(m_pSimulationLog, "MPI>\t Receive all messages (BODY_MESSAGE) from neighbours!" << std::endl;)
    if (m_nbRanks.size() == 0)
    {
        return;
    }

    // set the rank of the receiving message automatically! inside the function!
    m_pProcComm->receiveMessageFromRanks(m_message, m_nbRanks, m_message.m_tag);
    LOGBC(m_pSimulationLog, "MPI>\t Receive finished!" << std::endl;)

    // Wait for all sends to complete, Important because we issue a nonblocking send in sendMessagesToNeighbours
    m_pProcComm->waitForAllNeighbourSends();
}

void BodyCommunicator::cleanUp()
{
    LOGBC(m_pSimulationLog, "--->\t CleanUp Routine " << std::endl;)
    // Delete all bodies in the list
    for (auto* body : m_localBodiesToDelete)
    {
        LOGASSERTMSG(body->m_pBodyInfo->m_isRemote == false, m_pSimulationLog, "Body to delete is not a local body?!");

        for (auto& rankToFlags : body->m_pBodyInfo->m_neighbourRanks)
        {
            if (rankToFlags.second.m_inNeighbourMap == true)
            {
                auto* nbData = m_nbDataMap.getNeighbourData(rankToFlags.first);
                GRSF_ASSERTMSG(nbData, "No neighbour data for rank " << rankToFlags.first)

                bool res = nbData->eraseLocalBodyData(body);

                LOGASSERTMSG(res,
                             m_pSimulationLog,
                             "This local body with id: " << RigidBodyId::getBodyIdString(body)
                                                         << " could not be deleted in neighbour data rank: "
                                                         << rankToFlags.first);
            }
        }
        LOGBC(m_pSimulationLog,
              "--->\t Deleting body with id: " << RigidBodyId::getBodyIdString(body) << "@" << body << std::endl;)

        bool res = m_globalLocal.deleteBody(body);
        LOGASSERTMSG(res == true,
                     m_pSimulationLog,
                     "Remote Body with id: " << RigidBodyId::getBodyIdString(body)
                                             << " could not be deleted in m_globalRemote!");
        // FROM now it needs to be sure that this body is no where else in the system anymore!
    }

    m_localBodiesToDelete.clear();
}

void BodyCommunicator::printAllNeighbourRanks()
{
    for (auto* body : m_globalLocal)
    {
        LOGASSERTMSG(body->m_pBodyInfo,
                     m_pSimulationLog,
                     "Body info for local body with id: " << (body)->m_id << " does not exist!");
        LOGASSERTMSG(
            body->m_pBodyInfo->m_isRemote == false, m_pSimulationLog, "Local body to delete is not a local body?!");
        LOGBC(m_pSimulationLog, "BodyInfoRanks for body: " << RigidBodyId::getBodyIdString(body) << std::endl)
        for (auto& rankToFlags : body->m_pBodyInfo->m_neighbourRanks)
        {
            LOGBC(m_pSimulationLog, "(" << rankToFlags.first << "," << rankToFlags.second.m_overlaps << "), ")
        }
        LOGBC(m_pSimulationLog, std::endl)
    }
}
