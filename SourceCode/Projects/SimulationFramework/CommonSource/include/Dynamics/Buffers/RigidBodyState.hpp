#ifndef RigidBodyState_hpp
#define RigidBodyState_hpp

#include "TypeDefs.hpp"
#include "AssertionDebug.hpp"

#include "RigidBodyId.hpp"

class RigidBodyState;
namespace Interpolate {
    template<typename PREC>
    void lerp( const RigidBodyState & A, const RigidBodyState & B, RigidBodyState & X, PREC factor);
};


/**
* @ingroup StatesAndBuffers
* @brief This represents a dynamic state of a rigid body.
*/
class RigidBodyState {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    DEFINE_LAYOUT_CONFIG_TYPES

    RigidBodyState(){
        m_id = 0;
        m_u.setZero();
        m_q.setZero();
    };

    RigidBodyState(const RigidBodyIdType & id):m_id(id) {
        m_u.setZero();
        m_q.setZero();
    };

    RigidBodyState & operator =(const RigidBodyState& state) = default;

    template<typename TRigidBody>
    inline void applyBody( const TRigidBody  * body) {
        m_id = body->m_id;
        m_q = body->get_q();
        m_u = body->get_u();
    }

    friend void Interpolate::lerp<PREC>( const RigidBodyState & A, const RigidBodyState & B, RigidBodyState & X,PREC factor);
    RigidBodyIdType m_id;
    VectorQBody	m_q; ///< These are the generalized coordinates \f$\mathbf{q}\f$ of a rigid body.
    VectorUBody	m_u; ///< These are the generalized velocities \f$\mathbf{u}\f$ of a rigid body.
};


namespace Interpolate{
    template<typename PREC>
    void lerp( const RigidBodyState & A, const RigidBodyState & B, RigidBodyState & X, PREC factor) {
        X.m_q = A.m_q + factor*(B.m_q - A.m_q);
        X.m_u = A.m_u + factor*(B.m_u - A.m_u);
    };
};

#endif
