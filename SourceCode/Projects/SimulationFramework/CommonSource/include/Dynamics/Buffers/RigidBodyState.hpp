#ifndef RigidBodyState_hpp
#define RigidBodyState_hpp

#include "TypeDefs.hpp"
#include "AssertionDebug.hpp"

#include "RigidBodyId.hpp"

#include "AdditionalBodyData.hpp"

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
    friend void Interpolate::lerp<PREC>( const RigidBodyState & A, const RigidBodyState & B, RigidBodyState & X,PREC factor);
    RigidBodyIdType m_id;
    VectorQBody	m_q; ///< These are the generalized coordinates \f$\mathbf{q}\f$ of a rigid body.
    VectorUBody	m_u; ///< These are the generalized velocities \f$\mathbf{u}\f$ of a rigid body.

public:
    RigidBodyState(){
        m_id = 0;
        m_u.setZero();
        m_q.setZero();
    };

    RigidBodyState(const RigidBodyIdType & id):m_id(id){
        m_u.setZero();
        m_q.setZero();
    };

    template<typename TRigidBody>
    RigidBodyState( const TRigidBody  * body){
        applyBody(body);
    }

    ~RigidBodyState(){}

    RigidBodyState & operator =(const RigidBodyState& state) = default;

    template<typename TRigidBody>
    inline void applyBody( const TRigidBody  * body) {
        m_id = body->m_id;
        m_q  = body->get_q();
        m_u  = body->get_u();
    }

    auto getPosition() -> decltype(m_q.template head<3>()) {
        return m_q.template head<3>();
    }

    auto getPosition() const -> decltype(m_q.template head<3>()) {
        return m_q.template head<3>();
    }

    auto getVelocityTrans() -> decltype(m_u.template head<3>()){
        return m_u.template head<3>();
    }

    auto getVelocityTrans() const -> decltype(m_u.template head<3>()) {
        return m_u.template head<3>();
    }

};

class RigidBodyStateAdd : public RigidBodyState{
public:
    RigidBodyStateAdd(): RigidBodyState(), m_data(nullptr) {}

    RigidBodyStateAdd(const RigidBodyIdType & id, AdditionalBodyData::TypeEnum e )
    : RigidBodyState(id)
    {
            m_data = AdditionalBodyData::create(e);
    }

    AdditionalBodyData::Bytes * m_data = nullptr;

    ~RigidBodyStateAdd(){
        if(m_data){
            delete m_data;
        }
    }
};

namespace Interpolate{
    template<typename PREC>
    void lerp( const RigidBodyState & A, const RigidBodyState & B, RigidBodyState & X, PREC factor) {
        X.m_q = A.m_q + factor*(B.m_q - A.m_q);
        X.m_u = A.m_u + factor*(B.m_u - A.m_u);
    };
};

#endif
