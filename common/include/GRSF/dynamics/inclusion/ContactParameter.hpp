// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_inclusion_ContactParameter_hpp
#define GRSF_dynamics_inclusion_ContactParameter_hpp

#include "GRSF/common/TypeDefs.hpp"

#include "GRSF/dynamics/inclusion/ContactModels.hpp"

/**
* @ingroup Contact
* @brief This is the ContactParameter class, which stores the contact parameters.
* For  UCF
*    m_params[0];	///< The contact restitution coefficient in normal direction, \f$\epsilon_N\f$.
*    m_params[1];	///< The contact restitution coefficiend in tangential direction, \f$\epsilon_T\f$.
*    m_params[2];			///< The friction coefficient, \f$\mu\f$.
*
* For  UCFD
*    m_params[0];	///< The contact restitution coefficient in normal direction, \f$\epsilon_N\f$.
*    m_params[1];	///< The contact restitution coefficiend in tangential direction, \f$\epsilon_T\f$.
*    m_params[2];			///< The friction coefficient, \f$\mu\f$.
*    m_params[3];			///< The inverse damping constant for the unilateral contact \f$dinv_N\f$.
*    m_params[4];			///< The inverse damping constant for the frictional contact \f$dinv_T\f$.
* For  UCFDD
*    m_params[0];	///< The contact restitution coefficient in normal direction, \f$\epsilon_N\f$.
*    m_params[1];	///< The contact restitution coefficiend in tangential direction, \f$\epsilon_T\f$.
*    m_params[2];			///< The friction coefficient, \f$\mu\f$.
*    m_params[3];			///< The inverse damping constant for the unilateral contact \f$dinv_N\f$.
*    m_params[4];			///< The fixed inverse damping constant for the frictional contact \f$dinv_{TFix}\f$ if
* \f$|lambda_N| \leq epsilon\f$
*    m_params[5];			///< The maximum slipping velocity for frictional contact. \f$dinv_T = gamma_{max} / (\mu
* lambda_N) \f$
*    m_params[6];           ///< The epsilon to determine when to set the fixed damping constant \f$ dinv_TFix \f$
* @{ */
struct ContactParameter
{
    DEFINE_LAYOUT_CONFIG_TYPES

    ContactModels::Enum m_contactModel;
    std::vector<PREC>   m_params;

    ContactParameter(ContactModels::Enum e, std::initializer_list<PREC> it) : m_contactModel(e), m_params(it)
    {
    }
    ContactParameter() : m_contactModel(ContactModels::Enum::UCF), m_params{0.5, 0.5, 0.3}
    {
    }

    /** Copy constructors
    * not for ContactParameter a(ContactParameter::create_UCF_ConctactModel(...) );
    */
    ContactParameter(const ContactParameter& c) : m_contactModel(c.m_contactModel), m_params(c.m_params)
    {
    }

    /** Move constructors (if temporary is assigned to a new ContactParameter)
    * ContactParameter a(ContactParameter::create_UCF_ConctactModel(...) );
    */
    ContactParameter(ContactParameter&& c) : m_contactModel(c.m_contactModel), m_params(std::move(c.m_params))
    {
    }

    /**
    * Assignment operator
    */
    ContactParameter& operator=(const ContactParameter& c)
    {
        if (this != &c)
        {
            m_contactModel = c.m_contactModel;
            m_params       = c.m_params;
        }
        return *this;
    }

    /**
    * Move assignment operator
    */
    ContactParameter& operator=(ContactParameter&& c)
    {
        if (this != &c)
        {
            m_contactModel = std::move(c.m_contactModel);
            m_params       = std::move(c.m_params);  // move the container
        }
        return *this;
    }

    /**
    *
    * Here , never use std::move to move automatic objects out of function!,
    * the move constructor is called here to init the return value
    */
    static ContactParameter createParams_UCF_ContactModel(PREC epsN, PREC epsT, PREC mu)
    {
        return ContactParameter(ContactModels::Enum::UCF, {epsN, epsT, mu});
    }

    static ContactParameter createParams_UCFD_ContactModel(PREC epsN, PREC epsT, PREC mu, PREC dinv_N, PREC dinv_T)
    {
        return ContactParameter(ContactModels::Enum::UCFD, {epsN, epsT, mu, dinv_N, dinv_T});
    }
    static ContactParameter createParams_UCFDD_ContactModel(
        PREC epsN, PREC epsT, PREC mu, PREC dinv_N, PREC dinv_TFix, PREC gammaMax, PREC epsilon)
    {
        return ContactParameter(ContactModels::Enum::UCFD, {epsN, epsT, mu, dinv_N, dinv_TFix, gammaMax, epsilon});
    }
};

/** @} */

#endif
