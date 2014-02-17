﻿#ifndef ContactParameterMap_hpp
#define ContactParameterMap_hpp

#include <vector>
#include <algorithm>

#include <boost/tuple/tuple.hpp>
#include <boost/functional/hash.hpp>
#include <boost/unordered_map.hpp>

#include "TypeDefs.hpp"

#include "ContactParams.hpp"

/**
* @ingroup Contact
* @brief This is the ContactParameterMap class, which basically stores a diagonal matrix which maps a Material/Material to the corresponding ContactParams.
*/
/** @{ */
class ContactParameterTag ;

class ContactParameterTagHash : std::unary_function<ContactParameterTag, std::size_t> {
public:
    std::size_t operator()(ContactParameterTag const& c) const;
};


class ContactParameterTag {
public:

    typedef boost::tuples::tuple<unsigned int, unsigned int> ContactParameterTuple;
    ContactParameterTag(){ ContactParameterTag(0,0);};
    ContactParameterTag( unsigned int materialid1, unsigned int materialid2);

    bool operator==(ContactParameterTag const& c2) const;

    friend class ContactParamterTagHash;

    /**
    * This tuple builds up the hash. It consists of: 2 material ids
    **/
    ContactParameterTuple m_tag;
private:

};




class ContactParameterMap {
public:

    DEFINE_LAYOUT_CONFIG_TYPES

    typedef boost::unordered_map<ContactParameterTag, ContactParams, ContactParameterTagHash > ContactParameterMapType;

    ContactParameterMap();

    void setStandardValues(const ContactParams & params){ m_std_values = params;}

    /**
    * Adds parameters for one contact defined by two materials!
    */
    //bool addContactParameter(typename RigidBodyType::BodyMaterialType material1, typename RigidBodyType::BodyMaterialType  material2, const ContactParams & params);

    /**
    * @brief Gets the ContactParams for the material pair.
    * @param material1 The first material.
    * @param material2 The second material.
    * @return The reference to the ContactParams which corresponds to this kind of contact meterial pair.
    */
    //ContactParams & getContactParams(typename RigidBodyType::BodyMaterialType material1, typename RigidBodyType::BodyMaterialType  material2);

    template<typename TBodyMaterial>
    bool addContactParameter(TBodyMaterial  material1,
                             TBodyMaterial  material2,
                             const ContactParams & params)
    {
        typename ContactParameterMapType::value_type pair(ContactParameterTag(material1,material2),params);

        std::pair<  typename ContactParameterMapType::iterator, bool> res = m_ContactParams.insert(pair);

        if(res.second){
            m_nMaterials++;
        }

        return res.second;
    }

    template<typename TBodyMaterial>
    ContactParams & getContactParams(  TBodyMaterial material1,
                                       TBodyMaterial material2){

        typename ContactParameterMapType::iterator it = m_ContactParams.find(ContactParameterTag(material1,material2));

        if(it != m_ContactParams.end()){
            return it->second;
        }
        ASSERTMSG(false,"ContactParams for id: "<< material1 <<" and " <<material2 << " not found"<<std::endl);
        return m_std_values;
    }

    ContactParameterMapType & getContactParamsVector(){ return m_ContactParams;}

private:
    unsigned int m_nMaterials; ///< The number of materials. Determined from the @ref RigidBody::BodyMaterialType enumeration.

    ContactParams m_std_values; ///< Some standart values: m_std_values.m_epsilon_N = 0.5, m_std_values.m_epsilon_T = 0.5, m_std_values.m_mu = 0.3 .
    ContactParameterMapType m_ContactParams; /// All contact parameters (except the standart value);

};
/** @} */






#endif
