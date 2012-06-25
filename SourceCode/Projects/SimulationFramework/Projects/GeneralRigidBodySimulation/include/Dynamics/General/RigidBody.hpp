#ifndef RigidBody_hpp
#define RigidBody_hpp

#include <Eigen/Dense>


#include <boost/shared_ptr.hpp>
#include <boost/variant.hpp>
#include <boost/serialization/variant.hpp>

#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include "MeshData.hpp"

#include "SphereGeometry.hpp"
#include "PlaneGeometry.hpp"
#include "HalfspaceGeometry.hpp"
#include "BoxGeometry.hpp"
#include "MeshGeometry.hpp"

#include "QuaternionHelpers.hpp"

/**
* @ingroup DynamicsGeneral
* @brief This is the RigidBody class, which describes a rigid body.
*/
/** @{ */
template<typename TLayoutConfig>
class RigidBody{
public:
  DEFINE_LAYOUT_CONFIG_TYPES_OF(TLayoutConfig)
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW


  enum BodyState{
     SIMULATED,
     ANIMATED,
     NOT_SIMULATED
  }; ///< Emuration which defines if the object is simulated, animated or not simulated (which means fixed, and does not take part in the dynamics).

  enum BodyMaterial{  STD_MATERIAL = 0,
     WOOD = 1,
     METAL = 2,
     GLAS = 3,
     END = 4
  }; ///< Enumeration describing the Material.

  RigidBody(){
    m_mass = 1;
    m_MassMatrixInv_diag.setZero();
    m_MassMatrix_diag.setZero();
    m_K_Theta_S.setIdentity();
    m_A_IK.setIdentity();
    m_id = -1;
    m_r_S.setZero();
    setQuaternionZero(m_q_KI);
    m_h_term_const.setZero();
    m_h_term.setZero();
    m_eState = NOT_SIMULATED;
    m_eMaterial = STD_MATERIAL;
  }; ///< Constructor which sets standart values.

  ~RigidBody(){
    //DECONSTRUCTOR_MESSAGE
  };

    boost::variant<
      boost::shared_ptr<SphereGeometry<PREC> >,
      boost::shared_ptr<HalfspaceGeometry<PREC> >,
      boost::shared_ptr<BoxGeometry<PREC> >,
      boost::shared_ptr<MeshGeometry<PREC> >
    > m_geometry; ///< A boost::variant which takes different geometry shared pointers.

   PREC m_mass; ///< The rigid body mass \f$m\f$ in \f$ \textrm{[kg]} \f$

   Vector3 m_K_Theta_S; ///< The rigid body inertia tensor in diagonal form, \f$ {_K}\mathbf{\Theta}_{S}\f$ in \f$ [\textrm{kg} \cdot \textrm{m}^2] \f$


  VectorUObj m_MassMatrix_diag; ///< The mass matrix which is diagonal, \f$ \textrm{diag}(m,m,m, \textrm{diag}({_K}\mathbf{\Theta}_{S})) \f$.
  VectorUObj m_MassMatrixInv_diag;
  VectorUObj m_h_term, m_h_term_const;

  Matrix33 m_A_IK; ///< The transformation matrix \f$ \mathbf{A}_{IK} \f$ from K frame to the I frame which is updated at each timestep.

  /**
  * These values are updated from TimeStepper, used to faster compute certain stuff in assembler
  * @{
  */
  Vector3 m_r_S;     ///< Vector resolved in the I frame from origin to the center of gravity, \f$ \mathbf{r}_S \f$.
  Quaternion m_q_KI; ///< Quaternion which represents a rotation from I to the K frame, \f$ \tilde{\mathbf{a}}_{KI} \f$.
  /** @} */

  unsigned int m_id; ///< This is the id of the body.

  BodyState m_eState; ///< The state of the body.

  BodyMaterial m_eMaterial; ///< The material.
};

/** @} */




#endif
