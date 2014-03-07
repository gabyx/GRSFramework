#ifndef ContactParameter_hpp
#define ContactParameter_hpp

#include "TypeDefs.hpp"

#include "ContactModels.hpp"

/**
* @ingroup Contact
* @brief This is the ContactParameter class, which stores the contact parameters.
* For  NCF_ContactModel
*    m_params[0];	///< The contact restitution coefficient in normal direction, \f$\epsilon_N\f$.
*    m_params[1];	///< The contact restitution coefficiend in tangential direction, \f$\epsilon_T\f$.
*    m_params[2];			///< The friction coefficient, \f$\mu\f$.
*
* For  NCFD_ContactModel
*    m_params[0];	///< The contact restitution coefficient in normal direction, \f$\epsilon_N\f$.
*    m_params[1];	///< The contact restitution coefficiend in tangential direction, \f$\epsilon_T\f$.
*    m_params[2];			///< The friction coefficient, \f$\mu\f$.
*    m_params[3];			///< The damping constant for the unilateral contact \f$d_N\f$.
*    m_params[4];			///< The damping constant for the frictional contact \f$d_T\f$.
/** @{ */
struct ContactParameter{

    DEFINE_LAYOUT_CONFIG_TYPES

    ContactParameter(ContactModels::ContactModelEnum e, std::initializer_list<PREC> it ): m_params(it){}
    ContactParameter(): m_contactModel(ContactModels::ContactModelEnum::NCF_ContactModel), m_params{0.5,0.5,0.3}{}

    std::vector<PREC> m_params;
    ContactModels::ContactModelEnum m_contactModel;

    /** Copy constructors
    * not for ContactParameter a(ContactParameter::create_NCF_ConctactModel(...) );
    */
    ContactParameter(const ContactParameter & c): m_contactModel(c.m_contactModel), m_params(c.m_params) {}


    /** Move constructors (if temporary is assigned to a new ContactParameter)
    * ContactParameter a(ContactParameter::create_NCF_ConctactModel(...) );
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
    * Move assignmenet operator
    */
    ContactParameter& operator=(ContactParameter && c){
        m_contactModel = c.m_contactModel;
        m_params = std::move(c.m_params);
    }

    /**
    *
    * Here , never use std::move to move automatic objects out of function!,
    * the move constructor is called here to init the return value
    */
    static ContactParameter createParams_NCF_ContactModel(PREC epsN, PREC epsT, PREC mu){
        return ContactParameter(ContactModels::ContactModelEnum::NCF_ContactModel, {epsN,epsT,mu} );
    }
};

/** @} */

#endif
