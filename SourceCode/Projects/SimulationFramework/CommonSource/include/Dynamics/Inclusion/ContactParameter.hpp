﻿#ifndef ContactParameter_hpp
#define ContactParameter_hpp

#include "TypeDefs.hpp"

#include "ContactModels.hpp"

/**
* @ingroup Contact
* @brief This is the ContactParameter class, which stores the contact parameters.
* For  UCF_ContactModel
*    m_params[0];	///< The contact restitution coefficient in normal direction, \f$\epsilon_N\f$.
*    m_params[1];	///< The contact restitution coefficiend in tangential direction, \f$\epsilon_T\f$.
*    m_params[2];			///< The friction coefficient, \f$\mu\f$.
*
* For  UCFD_ContactModel
*    m_params[0];	///< The contact restitution coefficient in normal direction, \f$\epsilon_N\f$.
*    m_params[1];	///< The contact restitution coefficiend in tangential direction, \f$\epsilon_T\f$.
*    m_params[2];			///< The friction coefficient, \f$\mu\f$.
*    m_params[3];			///< The inverse damping constant for the unilateral contact \f$invd_N\f$.
*    m_params[4];			///< The inverse damping constant for the frictional contact \f$invd_T\f$.
/** @{ */
struct ContactParameter{

    DEFINE_LAYOUT_CONFIG_TYPES

    ContactParameter(ContactModels::ContactModelEnum e, std::initializer_list<PREC> it ): m_params(it), m_contactModel(e){}
    ContactParameter(): m_contactModel(ContactModels::ContactModelEnum::UCF_ContactModel), m_params{0.5,0.5,0.3}{}

    std::vector<PREC> m_params;
    ContactModels::ContactModelEnum m_contactModel;

    /** Copy constructors
    * not for ContactParameter a(ContactParameter::create_UCF_ConctactModel(...) );
    */
    ContactParameter(const ContactParameter & c): m_contactModel(c.m_contactModel), m_params(c.m_params) {}


    /** Move constructors (if temporary is assigned to a new ContactParameter)
    * ContactParameter a(ContactParameter::create_UCF_ConctactModel(...) );
    */
    ContactParameter(ContactParameter && c): m_contactModel(c.m_contactModel), m_params( std::move(c.m_params) ) {}

    /**
    * Assignment operator
    */
    ContactParameter& operator=(const ContactParameter & c){
        m_contactModel = c.m_contactModel;
        m_params = c.m_params;
    }

    /**
    * Move assignment operator
    */
    ContactParameter& operator=(ContactParameter && c){
        m_contactModel = std::move(c.m_contactModel);
        m_params = std::move(c.m_params); // move the container
    }

    /**
    *
    * Here , never use std::move to move automatic objects out of function!,
    * the move constructor is called here to init the return value
    */
    static ContactParameter createParams_UCF_ContactModel(PREC epsN, PREC epsT, PREC mu){
        return ContactParameter(ContactModels::ContactModelEnum::UCF_ContactModel, {epsN,epsT,mu} );
    }

    static ContactParameter createParams_UCFD_ContactModel(PREC epsN, PREC epsT, PREC mu, PREC d_N, PREC d_T){
        return ContactParameter(ContactModels::ContactModelEnum::UCFD_ContactModel, {epsN,epsT,mu,d_N,d_T} );
    }

};

/** @} */

#endif
