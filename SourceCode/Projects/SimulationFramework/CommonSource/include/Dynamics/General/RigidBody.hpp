#ifndef RigidBody_hpp
#define RigidBody_hpp

#include <memory>
#include <boost/variant.hpp>
#include <boost/serialization/variant.hpp>

#include <sstream>

#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include "RigidBodyId.hpp"

#include "MeshData.hpp"

#include "SphereGeometry.hpp"
#include "PlaneGeometry.hpp"
#include "HalfspaceGeometry.hpp"
#include "BoxGeometry.hpp"
#include "MeshGeometry.hpp"

#include "FrontBackBuffer.hpp"

#include "QuaternionHelpers.hpp"

#include RigidBodySolverData_INCLUDE_FILE

/**
* @ingroup DynamicsGeneral
* @brief This is the RigidBody class, which describes a rigid body.
*/
/** @{ */

class RigidBodyBase{
public:

    DEFINE_RIGIDBODY_CONFIG_TYPES
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef RigidBodyBase AbsoluteBaseType; ///< The absolut base type where m_id is defined, for the rigid body container

    enum class BodyState: char {
     SIMULATED = 0,
     STATIC = 1,
     ANIMATED = 2,
     NSTATES=3 // Just to have the number of how many states are defined!
    }; ///< Emuration which defines if the object is simulated, animated or not simulated (which means fixed, and does not take part in the dynamics).

    typedef unsigned int BodyMaterialType;

    unsigned int m_globalGeomId; ///< The Id for the global geometry, if this is 0 then the geometry belongs to the body and gets deallocated, otherwise not

    typedef boost::variant<
      std::shared_ptr<const SphereGeometry >,
      std::shared_ptr<const HalfspaceGeometry > ,
      std::shared_ptr<const BoxGeometry >,
      std::shared_ptr<const MeshGeometry >
    > GeometryType;

    GeometryType m_geometry; ///< A boost::variant which takes different geometry shared pointers.

    VectorUBody get_u(){
        return m_pSolverData->m_uBuffer.m_back;
    }
    VectorQBody get_q(){
        VectorQBody r; r.head<3>() = m_r_S; r.tail<4>() = m_q_KI; return r;
    }

    PREC m_mass; ///< The rigid body mass \f$m\f$ in \f$ \textrm{[kg]} \f$
    Vector3 m_K_Theta_S; ///< The rigid body inertia tensor in diagonal form, \f$ {_K}\mathbf{\Theta}_{S}\f$ in \f$ [\textrm{kg} \cdot \textrm{m}^2] \f$

    VectorUBody m_MassMatrix_diag; ///< The mass matrix which is diagonal, \f$ \textrm{diag}(m,m,m, \textrm{diag}({_K}\mathbf{\Theta}_{S})) \f$.
    VectorUBody m_MassMatrixInv_diag;
    VectorUBody m_h_term;

    Matrix33 m_A_IK; ///< The transformation matrix \f$ \mathbf{A}_{IK} \f$ from K frame to the I frame which is updated at each timestep.

    /**
    * These values are updated from TimeStepper, used to faster compute certain stuff during the collision solving and inclusion solving.
    * @{
    */
    Vector3 m_r_S;     ///< Vector resolved in the I frame from origin to the center of gravity, \f$ \mathbf{r}_S \f$, at time t_s + deltaT/2.
    Quaternion m_q_KI; ///< Quaternion which represents a rotation from I to the K frame, \f$ \tilde{\mathbf{a}}_{KI} \f$,  at time t_s + deltaT/2.

    typedef RigidBodyId::Type RigidBodyIdType;
    const RigidBodyIdType m_id; ///< This is the id of the body.

    BodyState m_eState; ///< The state of the body.
    BodyMaterialType m_eMaterial; ///< The material id.

    unsigned char m_flags; ///< Different Flags which can be used during the timestep process, introduces because of MPI

    RigidBodySolverDataType * m_pSolverData; /// Simulated bodies have a solverData. For all others, animated and not simulated this pointer is zero!


    RigidBodyBase(const RigidBodyIdType & id): m_id(id){
        m_mass = 1;
        m_MassMatrixInv_diag.setZero();
        m_MassMatrix_diag.setZero();
        m_K_Theta_S.setIdentity();
        m_A_IK.setIdentity();

        m_r_S.setZero();
        setQuaternionZero(m_q_KI);
        m_h_term.setZero();
        m_eState = BodyState::STATIC;
        m_eMaterial = 0;
        m_pSolverData = nullptr;
        m_globalGeomId = 0;
    }; ///< Constructor which sets standart values.

    ~RigidBodyBase(){
        //DECONSTRUCTOR_MESSAGE
        if(m_pSolverData){delete m_pSolverData; m_pSolverData = nullptr;}
    };

};




  /** @} */



/** @} */




#endif
