#ifndef GRSF_Dynamics_Buffers_RigidBodyState_hpp
#define GRSF_Dynamics_Buffers_RigidBodyState_hpp

#include "GRSF/Common/TypeDefs.hpp"
#include "GRSF/Common/AssertionDebug.hpp"

#include "GRSF/Dynamics/General/RigidBodyId.hpp"

#include "GRSF/Dynamics/General/AdditionalBodyData.hpp"

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

    void setDisplacement(const Vector3 & pos, const Quaternion & q){
        m_q.template head<3>() = pos;
        m_q.template tail<4>() = q.coeffs();
    }

    void setVelocity(const Vector3 & trans, const Vector3 & omega){
        m_u.template head<3>() = trans;
        m_u.template tail<3>() = omega;
    }

    auto getPosition() -> decltype(m_q.template head<3>()) {
        return m_q.template head<3>();
    }
    auto getPosition() const -> const decltype(m_q.template head<3>()){
        return m_q.template head<3>();
    }

    auto getRotation() -> decltype(m_q.template tail<4>()) {
        return m_q.template tail<4>();
    }
    auto getRotation() const -> const decltype(m_q.template tail<4>()) {
        return m_q.template tail<4>();
    }

    auto getVelocityTrans() -> decltype(m_u.template head<3>()) {
        return m_u.template head<3>();
    }
    auto getVelocityTrans() const -> const decltype(m_u.template head<3>()) {
        return m_u.template head<3>();
    }
    auto  getVelocityRot() -> decltype(m_u.template tail<3>()){
        return m_u.template tail<3>();
    }
    auto  getVelocityRot() const -> const decltype(m_u.template tail<3>()){
       return m_u.template tail<3>();
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
