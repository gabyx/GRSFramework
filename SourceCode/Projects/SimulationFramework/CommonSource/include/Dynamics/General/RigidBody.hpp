#ifndef RigidBody_hpp
#define RigidBody_hpp

#include <boost/shared_ptr.hpp>
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

/**
* @ingroup DynamicsGeneral
* @brief This is the RigidBody class, which describes a rigid body.
*/
/** @{ */

class RigidBodySolverData{
    public:

        DEFINE_LAYOUT_CONFIG_TYPES

        RigidBodySolverData(): m_t(0){};

        ///< The actual time, which belongs to FrontBuffer and m_r_S and I_q_IK
        PREC m_t;
};



/** Class with  Data Structure for the Solver! */
class RigidBodySolverDataCONoG : public RigidBodySolverData {

    public:
    DEFINE_LAYOUT_CONFIG_TYPES
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    RigidBodySolverDataCONoG(): m_bInContactGraph(false){
        m_uBuffer.m_front.setZero();
        m_uBuffer.m_back.setZero();
    };

    ///< Get the actual velocity

    ///< Flag which determines if this body is in the contact graph!
    bool m_bInContactGraph;

    ///< Pointers into the right Front BackBuffer for the velocity which get iteratet in the InclusionSolverCONoG
    ///< The back buffer is the velocity before the prox iteration (over all nodes)
    ///< The front buffer is the velocity which is used to during ONE prox iteration
    FrontBackBuffer<VectorUObj,FrontBackBufferPtrType::NoPtr, FrontBackBufferMode::NoConst> m_uBuffer;

    void swapBuffer(){
        m_uBuffer.m_front.swap(m_uBuffer.m_back);
    }

    void reset(){
        m_uBuffer.m_front.setZero();
        m_bInContactGraph = false;
    };

};

class RigidBodyBase{
public:

    DEFINE_RIGIDBODY_CONFIG_TYPES
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum BodyState{
     SIMULATED,
     ANIMATED,
     NOT_SIMULATED
    }; ///< Emuration which defines if the object is simulated, animated or not simulated (which means fixed, and does not take part in the dynamics).

    typedef unsigned int BodyMaterialType;

    unsigned int m_globalGeomId; ///< The Id for the global geometry, if this is 0 then the geometry belongs to the body and gets deallocated, otherwise not

    typedef boost::variant<
      boost::shared_ptr<const SphereGeometry<PREC> >,
      boost::shared_ptr<const HalfspaceGeometry > ,
      boost::shared_ptr<const BoxGeometry >,
      boost::shared_ptr<const MeshGeometry >
    > GeometryType;

    GeometryType m_geometry; ///< A boost::variant which takes different geometry shared pointers.

    VectorUObj get_u(){
        return m_pSolverData->m_uBuffer.m_back;
    }
    VectorQObj get_q(){
        VectorQObj r; r.head<3>() = m_r_S; r.tail<4>() = m_q_KI; return r;
    }

    PREC m_mass; ///< The rigid body mass \f$m\f$ in \f$ \textrm{[kg]} \f$
    Vector3 m_K_Theta_S; ///< The rigid body inertia tensor in diagonal form, \f$ {_K}\mathbf{\Theta}_{S}\f$ in \f$ [\textrm{kg} \cdot \textrm{m}^2] \f$

    VectorUObj m_MassMatrix_diag; ///< The mass matrix which is diagonal, \f$ \textrm{diag}(m,m,m, \textrm{diag}({_K}\mathbf{\Theta}_{S})) \f$.
    VectorUObj m_MassMatrixInv_diag;
    VectorUObj m_h_term, m_h_term_const;

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
        m_h_term_const.setZero();
        m_h_term.setZero();
        m_eState = NOT_SIMULATED;
        m_eMaterial = 0;
        m_pSolverData = NULL;
        m_globalGeomId = 0;
    }; ///< Constructor which sets standart values.

    ~RigidBodyBase(){
        //DECONSTRUCTOR_MESSAGE
        if(m_pSolverData){delete m_pSolverData; m_pSolverData = NULL;}
    };
};

  /** @} */



/** @} */




#endif
