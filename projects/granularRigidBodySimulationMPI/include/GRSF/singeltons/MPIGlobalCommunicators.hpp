// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_singeltons_MPIGlobalCommunicators_hpp
#define GRSF_singeltons_MPIGlobalCommunicators_hpp

#include <mpi.h>

#include "GRSF/common/Singleton.hpp"
#include "GRSF/dynamics/general/MPICommunicatorId.hpp"

namespace MPILayer{
/**
* @brief Collects the top most important communicators which are fixed during the simulation
* e.g. SIM_COMM and WORLD_COMM usually
*/

class MPIGlobalCommunicators : public Utilities::Singleton<MPIGlobalCommunicators>{
    public:
        MPIGlobalCommunicators(){
            // Add MPI WORLD
            m_communicators.insert(std::make_pair(static_cast<unsigned int>(MPICommunicatorId::WORLD_COMM),
                                                  MPI_COMM_WORLD ));

        }
        inline void addCommunicator(MPICommunicatorId commId, MPI_Comm comm){
            auto res = m_communicators.insert(std::make_pair(static_cast<unsigned int>(commId),comm));
            if(!res.second){
                ERRORMSG("Communicator with id: " << static_cast<unsigned int>(commId) << " already exists!")
            }
        }

        inline MPI_Comm getCommunicator(MPICommunicatorId commId){
            auto res = m_communicators.find(static_cast<unsigned int>(commId));
            if(res == m_communicators.end()){
                ERRORMSG("Communicator with id: " << static_cast<unsigned int>(commId) <<" does not exists!")
            }
            return (res->second);
        }

    private:
        std::map<unsigned int, MPI_Comm> m_communicators;
};
};

#endif
