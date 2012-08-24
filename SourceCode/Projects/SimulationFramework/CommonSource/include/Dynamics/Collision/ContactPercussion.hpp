#ifndef ContactPercussion_hpp
#define ContactPercussion_hpp

/**
* @ingroup Contact
* @brief This is the ContactPercussion class, which stores the percussions for one contact.
*/
/** @{ */
template<typename TLayoutConfig>
class ContactPercussion {
public:
   DEFINE_LAYOUT_CONFIG_TYPES_OF(TLayoutConfig)
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

   ContactPercussion(unsigned int dim){
      m_P.setZero(dim);
      m_bUsed = false;
   }

   VectorDyn m_P; ///< The contact percussion, e.g \f$[\lambda_N,\lambda_{T1},\lambda_{T2}]\f$

   unsigned int m_bUsed; ///< A flag which defines if this Percussion has been used in the current timestep.
};
/** @} */
#endif
