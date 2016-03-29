// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_general_MPIMessageTag_hpp
#define GRSF_dynamics_general_MPIMessageTag_hpp

/**
*    Important struct to define all MPI message tags used in this framework!
*/


namespace MPILayer{
    enum class MPIMessageTag: unsigned int {
        GENERICMESSAGE = 1 << 0,
        STDSTRING =      1 << 1,
        BODY_MESSAGE =   1 << 2,
        EXTERNALCONTACTS_MESSAGE = 1 << 3,
        SPLITBODYFACTOR_MESSAGE  = 1 << 4,
        SPLITBODYUPDATE_MESSAGE  = 1 << 5,
        SPLITBODYSOLUTION_MESSAGE = 1<< 6,
        TOPOLOGYBUILDER_POINTGATHER = 1<<7,
        TOPOLOGYBUILDER_RESULTS = 1<<8
    };
};

#endif
