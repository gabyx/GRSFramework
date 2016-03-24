// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_general_RigidBodySolverDataMPI_hpp
#define GRSF_dynamics_general_RigidBodySolverDataMPI_hpp

#include "GRSF/common/TypeDefs.hpp"

#include "GRSF/dynamics/general/RigidBodySolverData.hpp"

/** Class with  Data Structure for the Solver! */
class RigidBodySolverDataCONoGMPI : public RigidBodySolverDataCONoG {

    public:
    DEFINE_LAYOUT_CONFIG_TYPES
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    RigidBodySolverDataCONoGMPI(): RigidBodySolverDataCONoG(){
        m_multiplicityWeight = 1.0;
        m_multiplicity = 1; // If one this body is not split, if >1 then it is split
    };

    /** Dont change these values directly, do this over the two functions
        RigidBodyFunctions::changeBodyToSplitWeighting and
        RigidBodyFunctions::changeBodyToNormalWeighting
    */
    PREC m_multiplicityWeight;  ///< This is the actual weight factor which is used by this process, to scale the mass matrix and the h vector!
    unsigned int m_multiplicity;  ///< This is the factor in how many virtual parts the body is split during the global inlcusion solving process

};

#endif

