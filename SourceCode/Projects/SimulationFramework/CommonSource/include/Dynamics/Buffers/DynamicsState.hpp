#ifndef DYNAMICS_STATE_HPP
#define DYNAMICS_STATE_HPP

#include <vector>

#include "TypeDefs.hpp"
#include "AssertionDebug.hpp"

#include "QuaternionHelpers.hpp"

//Prototype
class DynamicsState;
class RigidBodyState;

namespace Interpolate {
    template<typename PREC>
    void lerp( const RigidBodyState & A, const RigidBodyState & B, RigidBodyState & X, PREC factor);
    template<typename PREC>
    void lerp( const DynamicsState & A, const DynamicsState & B, DynamicsState & X, PREC factor);
};


/**
* @ingroup StatesAndBuffers
* @brief This represents a dynamic state of a rigid body.
*/
class RigidBodyState {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    DEFINE_LAYOUT_CONFIG_TYPES

    RigidBodyState() {
        m_u.setZero();
        m_q.setZero();
    };

    RigidBodyState & operator =(const RigidBodyState& state) {
        m_u = state.m_u;
        m_q = state.m_q;
        return *this;
    }

    friend void Interpolate::lerp<PREC>( const RigidBodyState & A, const RigidBodyState & B, RigidBodyState & X,PREC factor);

    VectorQObj	m_q; ///< These are the generalized coordinates \f$\mathbf{q}\f$ of a rigid body.
    VectorUObj	m_u; ///< These are the generalized velocities \f$\mathbf{u}\f$ of a rigid body.
};



/**
* @ingroup StatesAndBuffers
* @brief This represents a dynamic state of the dynamics system.
*/
class DynamicsState {
public:
    DEFINE_LAYOUT_CONFIG_TYPES
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    DynamicsState(const unsigned int nSimBodies):
        m_nSimBodies(nSimBodies)
    {
        m_t = 0.0;
        m_StateType = NONE;
        m_SimBodyStates.assign(nSimBodies,RigidBodyState());
    };

    ~DynamicsState(){};

    DynamicsState & operator =(const DynamicsState& state){
        m_StateType = state.m_StateType;
        m_t = state.m_t;
        ASSERTMSG( m_SimBodyStates.size() == state.m_SimBodyStates.size(), "DynamicsState:: Size mismatch!" );
        for(int i=0; i<state.m_SimBodyStates.size(); i++) {
            m_SimBodyStates[i] = state.m_SimBodyStates[i];
        }
        return *this;
    }

    friend void Interpolate::lerp<PREC>( const DynamicsState & A, const DynamicsState & B, DynamicsState & X,
                                        PREC factor);

    PREC	m_t; ///< The time in \f$[s]\f$

    typedef std::vector< RigidBodyState > RigidBodyStateListType;

    RigidBodyStateListType  m_SimBodyStates; ///< A vector comprising of all rigid body states of the system for simulated objects.
    RigidBodyStateListType  m_AniBodyStates; ///< A vector comprising of all rigid body states of the system for animated objects.

    const unsigned int m_nSimBodies;

    enum {NONE = 0, STARTSTATE=1, ENDSTATE = 2} m_StateType;
};



template<typename PREC>
void Interpolate::lerp( const RigidBodyState & A, const RigidBodyState & B, RigidBodyState & X, PREC factor) {
    X.m_q = A.m_q + factor*(B.m_q - A.m_q);
    X.m_u = A.m_u + factor*(B.m_u - A.m_u);
};

template<typename PREC>
void Interpolate::lerp( const DynamicsState & A, const DynamicsState & B, DynamicsState & X, PREC factor) {
    ASSERTMSG(A.m_nSimBodies == B.m_nSimBodies &&  B.m_nSimBodies == X.m_nSimBodies ,"Wrong number of bodies!");
    X.m_t = X.m_t = A.m_t + factor*(B.m_t - A.m_t);
    for(int i=0; i<A.m_SimBodyStates.size(); i++) {
        lerp(A.m_SimBodyStates[i],B.m_SimBodyStates[i],X.m_SimBodyStates[i],factor);
    }
};




#endif
