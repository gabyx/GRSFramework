#ifndef GRSF_Dynamics_General_RigidBody_hpp
#define GRSF_Dynamics_General_RigidBody_hpp

#include <memory>
#include <boost/variant.hpp>
#include <boost/serialization/variant.hpp>

#include <sstream>

#include "GRSF/Common/TypeDefs.hpp"
#include "GRSF/Common/LogDefines.hpp"

#include "GRSF/Dynamics/General/RigidBodyId.hpp"
#include "GRSF/Dynamics/Buffers/FrontBackBuffer.hpp"
#include "GRSF/Dynamics/General/QuaternionHelpers.hpp"

#include "GRSF/Dynamics/Collision/Geometries.hpp"
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

    using AbsoluteBaseType = RigidBodyBase; ///< The absolut base type where m_id is defined, for the rigid body container

    enum class BodyMode: char {
     SIMULATED = 0,
     STATIC = 1,
     ANIMATED = 2,
     NSTATES=3 // Just to have the number of how many states are defined!
    }; ///< Emuration which defines if the object is simulated, animated or not simulated (which means fixed, and does not take part in the dynamics).

    using BodyMaterialType = unsigned int;

    using GlobalGeomIdType = unsigned int;
    GlobalGeomIdType m_globalGeomId; ///< The Id for the global geometry, if this is 0 then the geometry belongs to the body and gets deallocated, otherwise not


    /** Define Geometry Ptr Types */
    using SphereGeomPtrType     = std::shared_ptr<const SphereGeometry >;
    using HalfspaceGeomPtrType  = std::shared_ptr<const HalfspaceGeometry>;
    using BoxGeomPtrType        = std::shared_ptr<const BoxGeometry >;
    using MeshPtrType           = std::shared_ptr<const MeshGeometry >;

    typedef boost::variant<
      SphereGeomPtrType, HalfspaceGeomPtrType , BoxGeomPtrType, MeshPtrType
    > GeometryType;

    GeometryType m_geometry; ///< A boost::variant which takes different geometry shared pointers.

    inline VectorUBody get_u() const{
        if(m_pSolverData){
            return m_pSolverData->m_uBuffer.m_back;
        }
        VectorUBody u; u.setZero();
        return u;
    }
    inline VectorQBody get_q() const{
        VectorQBody r; r.head<3>() = m_r_S; r.tail<4>() = m_q_KI.coeffs(); return r;
    }

    template<bool setTrafoMatrix = false,typename TRigidBodyState>
    inline void applyBodyState(const TRigidBodyState & s){
        m_r_S = s.m_q.template head<3>();
        m_q_KI = s.m_q.template tail<4>();

        if(setTrafoMatrix ){
            //QuaternionHelpers::setRotFromQuaternion(m_q_KI,m_A_IK);
            m_A_IK = m_q_KI.matrix();
        }

        if(m_pSolverData){
            m_pSolverData->m_uBuffer.m_back = s.m_u;
        }
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

    using RigidBodyIdType = RigidBodyId::Type;
    const RigidBodyIdType m_id; ///< This is the id of the body.

    BodyMode m_eMode; ///< The state of the body.
    BodyMaterialType m_eMaterial; ///< The material id.

    unsigned char m_flags; ///< Different Flags which can be used during the timestep process, introduced because of MPI

    BodySolverDataType * m_pSolverData; /// Simulated bodies have a solverData. For all others, animated and not simulated this pointer is zero!


    RigidBodyBase(const RigidBodyIdType & id): m_id(id){
        m_mass = 1;
        m_MassMatrixInv_diag.setZero();
        m_MassMatrix_diag.setZero();
        m_K_Theta_S.setIdentity();
        m_A_IK.setIdentity();

        m_r_S.setZero();
        m_q_KI.setIdentity();//QuaternionHelpers::setQuaternionZero(m_q_KI);
        m_h_term.setZero();
        m_eMode = BodyMode::STATIC;
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
