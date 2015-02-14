#ifndef GRSF_Dynamics_Collision_ContactPercussion_hpp
#define GRSF_Dynamics_Collision_ContactPercussion_hpp

#include "GRSF/Dynamics/Inclusion/ContactModels.hpp"

/**
* @ingroup Contact
* @brief This is the ContactPercussion class, which caches the percussions for one contact.
*/
/** @{ */
class ContactPercussion {
public:

    DEFINE_LAYOUT_CONFIG_TYPES
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ContactPercussion(unsigned int dim, bool used) : m_Lambda(dim), m_used(true) {};

    bool m_used; ///< A flag which defines if this Percussion has been used in the current timestep.
    VectorDyn m_Lambda; ///< The contact percussion, e.g \f$[\Lambda_N,\Lambda_{T1},\Lambda_{T2}]\f$
};
/** @} */
#endif
