#ifndef GRSF_Dynamics_Inclusion_ContactParameterMap_hpp
#define GRSF_Dynamics_Inclusion_ContactParameterMap_hpp

#include <vector>
#include <algorithm>

#include <tuple>
#include <unordered_map>

#include "GRSF/Common/TypeDefs.hpp"
#include "GRSF/Common/AssertionDebug.hpp"

#include "GRSF/Dynamics/Inclusion/ContactParameter.hpp"

#include "GRSF/Common/TupleHash.hpp"

#include RigidBody_INCLUDE_FILE

/**
* @ingroup Contact
* @brief This is the ContactParameterMap class, which basically stores a diagonal matrix which maps a Material/Material to the corresponding ContactParameter.
*/
/** @{ */


class ContactParameterTagHash;

class ContactParameterTag {
public:

    using Tuple = std::tuple<unsigned int, unsigned int>;
    ContactParameterTag( unsigned int materialid1, unsigned int materialid2);

    bool operator==(ContactParameterTag const& c2) const;

    friend class ContactParameterTagHash;
private:


    /**
    * This tuple builds up the hash. It consists of: 2 material ids
    **/
    Tuple m_tag{};
};


class ContactParameterTagHash : std::unary_function<ContactParameterTag, std::size_t> {
public:
    std::size_t operator()(ContactParameterTag const& c) const{
            TupleHash::hash<typename ContactParameterTag::Tuple> hasher;
            return hasher(c.m_tag);
    }
};




class ContactParameterMap {
public:

    DEFINE_RIGIDBODY_CONFIG_TYPES

    using MaterialIdType = typename RigidBodyType::BodyMaterialType;

    using ContactParameterMapType = std::unordered_map<ContactParameterTag, ContactParameter, ContactParameterTagHash >;

    ContactParameterMap();

    void setStandardValues(const ContactParameter & params){ m_std_values = params;}

    /**
    * @brief Gets the ContactParameter for the material pair.
    * @param material1 The first material.
    * @param material2 The second material.
    * @return The reference to the ContactParameter which corresponds to this kind of contact meterial pair.
    */

    bool addContactParameter(const MaterialIdType & material1,
                             const MaterialIdType & material2,
                             const ContactParameter & params)
    {
        auto res = m_ContactParams.emplace(std::make_pair(ContactParameterTag(material1,material2),params));
        return res.second;
    }

    ContactParameter & getContactParams(const MaterialIdType & material1,
                                        const MaterialIdType & material2){

        auto it = m_ContactParams.find(ContactParameterTag(material1,material2));

        if(it != m_ContactParams.end()){
            return it->second;
        }
        ASSERTMSG(false,"ContactParameter for id: "<< material1 <<" and " <<material2 << " not found"<<std::endl);
        return m_std_values;
    }

    ContactParameterMapType & getContactParams(){ return m_ContactParams;}

private:
    ContactParameter m_std_values; ///< Some standart values: m_std_values.m_epsilon_N = 0.5, m_std_values.m_epsilon_T = 0.5, m_std_values.m_mu = 0.3 .
    ContactParameterMapType m_ContactParams; /// All contact parameters (except the standart value);

};
/** @} */







#endif
