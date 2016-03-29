// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_general_MPICommunicatorId_hpp
#define GRSF_dynamics_general_MPICommunicatorId_hpp

/**
* Identifiers for common used communicators
*/
namespace MPILayer{
    enum class MPICommunicatorId: unsigned int{
            WORLD_COMM = 0,
            SIM_COMM = 1,                 ///< All processes which take part in the simulation

            /** Only processes which take part in the convergence comm
            * The underlying communicator for this Id can be different for each process!
            */
            INCLUSION_CONVERGENCE_COMM = 2
    };
};

#endif
