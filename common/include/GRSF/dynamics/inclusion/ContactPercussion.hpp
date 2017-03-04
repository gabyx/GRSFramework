// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_inclusion_ContactPercussion_hpp
#define GRSF_dynamics_inclusion_ContactPercussion_hpp

#include "GRSF/dynamics/inclusion/ContactModels.hpp"

/**
* @ingroup Contact
* @brief This is the ContactPercussion class, which caches the percussions for one contact.
*/
/** @{ */
class ContactPercussion
{
public:
    DEFINE_LAYOUT_CONFIG_TYPES
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ContactPercussion(unsigned int dim, bool used) : m_used(true), m_Lambda(dim){};

    bool m_used;         ///< A flag which defines if this Percussion has been used in the current timestep.
    VectorDyn m_Lambda;  ///< The contact percussion, e.g \f$[\Lambda_N,\Lambda_{T1},\Lambda_{T2}]\f$
};
/** @} */
#endif
