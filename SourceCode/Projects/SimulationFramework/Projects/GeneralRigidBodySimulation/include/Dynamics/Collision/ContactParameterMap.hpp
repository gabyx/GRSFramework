#ifndef ContactParameterMap_hpp
#define ContactParameterMap_hpp

#include <vector>
#include <algorithm>
#include "TypeDefs.hpp"

#include "ContactParams.hpp"

/**
* @ingroup Contact
* @brief This is the ContactParameterMap class, which basically stores a diagonal matrix which maps a Material/Material to the corresponding ContactParams.
*/
/** @{ */
template< typename TLayoutConfig>
class ContactParameterMap{

  DEFINE_LAYOUT_CONFIG_TYPES_OF(TLayoutConfig)

public:
  ContactParameterMap();

  /**
  * @brief Gets the ContactParams for the material pair.
  * @param material1 The first material.
  * @param material2 The second material.
  * @return The reference to the ContactParams which corresponds to this kind of contact meterial pair.
  */
  ContactParams<TLayoutConfig> & getContactParams(typename RigidBody<TLayoutConfig>::BodyMaterial material1, typename RigidBody<TLayoutConfig>::BodyMaterial  material2);
  std::vector<ContactParams<TLayoutConfig> > & getContactParamsVector();

private:
  unsigned int m_nMaterials; ///< The number of materials. Determined from the @ref RigidBody::BodyMaterial enumeration.

  ContactParams<TLayoutConfig> m_std_values; ///< Some standart values: m_std_values.m_epsilon_N = 0.5, m_std_values.m_epsilon_T = 0.5, m_std_values.m_mu = 0.3 .

  std::vector<ContactParams<TLayoutConfig> > m_ContactParams; ///< The matrix corresponding to the ContactParams for the different material pairs.

};
/** @} */

template< typename TLayoutConfig>
ContactParameterMap<TLayoutConfig>::ContactParameterMap()
{
   m_nMaterials = (int)RigidBody<TLayoutConfig>::END-(int)RigidBody<TLayoutConfig>::STD_MATERIAL;
  m_std_values.m_epsilon_N = 0.2;
  m_std_values.m_epsilon_T = 0.0;
  m_std_values.m_mu = 0.3;

  int nEntries = (m_nMaterials*(m_nMaterials+1))/2;
  m_ContactParams.assign( nEntries, m_std_values);


// Implement here, the specific params for material <-> material
// TODO

}


template< typename TLayoutConfig>
ContactParams<TLayoutConfig> & ContactParameterMap<TLayoutConfig>::getContactParams(typename RigidBody<TLayoutConfig>::BodyMaterial material1, typename RigidBody<TLayoutConfig>::BodyMaterial material2)
{

  unsigned int mat1 = (int)material1;
  unsigned int mat2 = (int)material2;
  
  int offset;
  // if in lower triangular part of matrix , swap
  if( mat1 > mat2 ){
     std::swap(mat1,mat2);
  }
    ASSERTMSG( mat1 < m_nMaterials && mat2 < m_nMaterials, " Error: Indexing error!");
    offset = (mat1 * m_nMaterials + mat2) - ((mat1 + 2)*(mat1 + 1))/2 + (mat1 + 1);
    ASSERTMSG( offset < m_ContactParams.size(), "Error: Indexing error!");
    return m_ContactParams[offset];

}

template< typename TLayoutConfig>
std::vector<ContactParams<TLayoutConfig> > & ContactParameterMap<TLayoutConfig>::getContactParamsVector()
{
  return m_ContactParams;
}



#endif