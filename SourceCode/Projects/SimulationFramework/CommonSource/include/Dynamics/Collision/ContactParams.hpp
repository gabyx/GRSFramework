#ifndef ContactParams_hpp
#define ContactParams_hpp

#include "TypeDefs.hpp"

/**
* @ingroup Contact
* @brief This is the ContactParams class, which stores the contact parameters.
*/
/** @{ */
template< typename TLayoutConfig>
struct ContactParams{

    DEFINE_LAYOUT_CONFIG_TYPES_OF(TLayoutConfig)

    ContactParams(double epsilon_N, double epsilon_T, double mu)
      : m_epsilon_N(epsilon_N), m_epsilon_T(epsilon_T),m_mu(mu)
    {};

    ContactParams()
      : m_epsilon_N(0.5), m_epsilon_T(0.5),m_mu(0.3)
    {};

    PREC m_epsilon_N;	///< The contact restitution coefficient in normal direction, \f$\epsilon_N\f$.
    PREC m_epsilon_T;	///< The contact restitution coefficiend in tangential direction, \f$\epsilon_T\f$.
    PREC m_mu;			///< The friction coefficient, \f$\mu\f$.
};

/** @} */

#endif