#ifndef DYNAMICS_STATE_HPP
#define DYNAMICS_STATE_HPP

#include <vector>
#include <unordered_map>

#include "TypeDefs.hpp"
#include "AssertionDebug.hpp"

#include "QuaternionHelpers.hpp"
#include "RigidBodyId.hpp"

//Prototype
class DynamicsState;
class DynamicsStateBase;
class RigidBodyState;

namespace Interpolate {
    template<typename PREC>
    void lerp( const RigidBodyState & A, const RigidBodyState & B, RigidBodyState & X, PREC factor);
    template<typename PREC>
    void lerp( const DynamicsStateBase & A, const DynamicsStateBase & B, DynamicsStateBase & X, PREC factor);
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

    RigidBodyState & operator =(const RigidBodyState& state) {
        m_id = state.m_id;
        m_u = state.m_u;
        m_q = state.m_q;
        return *this;
    }

    friend void Interpolate::lerp<PREC>( const RigidBodyState & A, const RigidBodyState & B, RigidBodyState & X,PREC factor);
    RigidBodyIdType m_id;
    VectorQBody	m_q; ///< These are the generalized coordinates \f$\mathbf{q}\f$ of a rigid body.
    VectorUBody	m_u; ///< These are the generalized velocities \f$\mathbf{u}\f$ of a rigid body.
};



/**
* @ingroup StatesAndBuffers
* @brief This represents a dynamic state of the dynamics system.
*/
class DynamicsStateBase{
public:

    DEFINE_LAYOUT_CONFIG_TYPES

    DynamicsStateBase():m_t(0.0),m_StateType(NONE){}

    PREC m_t; ///< The time in seconds
    enum {NONE = 0, STARTSTATE=1, ENDSTATE = 2} m_StateType;
    typedef std::vector< RigidBodyState > RigidBodyStateListType;

    friend void Interpolate::lerp<PREC>( const DynamicsStateBase & A, const DynamicsStateBase & B, DynamicsStateBase & X, PREC factor);

     /** Bodies (ids) sorted according to their insertion order!
     *   We use here a linear continous memory since this class acts as a buffer which get written by the simulation and read by the visualization
     */
    RigidBodyStateListType  m_SimBodyStates; ///< A vector comprising of all rigid body states of the system for simulated objects.
    RigidBodyStateListType  m_AniBodyStates; ///< A vector comprising of all rigid body states of the system for animated objects.
};

class DynamicsState: public DynamicsStateBase{
public:

    DynamicsState():DynamicsStateBase(),m_randomAccess(true){}

    template<typename RigidBodyIterator>
    DynamicsState(RigidBodyIterator itBegin, RigidBodyIterator itEnd)
    {
        initSimStates<true>(itBegin,itEnd);
    };

    ~DynamicsState(){};

    DynamicsState & operator=(const DynamicsState& state){
        m_StateType = state.m_StateType;
        m_t = state.m_t;
        m_randomAccess  = state.m_randomAccess;
        m_pIdToState    = state.m_pIdToState;
        m_SimBodyStates = state.m_SimBodyStates;
        m_AniBodyStates = state.m_AniBodyStates;
        return *this;
    }

    void reset(){
      m_StateType = DynamicsState::NONE;
      m_t = 0.0;
    }

    RigidBodyStateListType::size_type getNSimBodies() const {return m_SimBodyStates.size();}

    template<bool resetState, typename RigidBodyIterator>
    inline void initSimStates(RigidBodyIterator itBegin, RigidBodyIterator itEnd){
        m_randomAccess = true;
        m_t = 0.0;
        m_StateType = NONE;

        unsigned int nSimBodies = std::distance(itBegin,itEnd);
        ASSERTMSG(nSimBodies, "nSimBodies == 0");
        if(resetState){
            m_SimBodyStates.assign(nSimBodies,RigidBodyState());
        }else{
            m_SimBodyStates.resize(nSimBodies);
        }

        unsigned int i = 0;
        for(auto it = itBegin; it!= itEnd;it++){
            // Check for continuity in ids
            auto id = (*it)->m_id;
            if( m_randomAccess && std::next(it)!=itEnd && (*std::next(it))->m_id - (*it)->m_id != 1 ){
                m_randomAccess=false;
            }
            m_SimBodyStates[i].m_id = id;
            m_pIdToState.insert(std::make_pair(id,&m_SimBodyStates[i]));
        }
    }

    /** Access the SimState for a given Id*/
    RigidBodyState * getSimState(const RigidBodyIdType & id){
        if(m_randomAccess){
            auto bodyNr = RigidBodyId::getBodyNr(id);
            if( bodyNr < m_SimBodyStates.size()){
                return &m_SimBodyStates[bodyNr];
            }
        }else{
            auto it = m_pIdToState.find(id);
            if(it!=m_pIdToState.end()){
                return it->second;
            }
        }
        return nullptr;
    }

    bool hasRandomAccess(){return m_randomAccess;}

private:
    /** For access to specific ids:
    * if m_randomAccess is false, then the ids in RigidBodyStateListType are not continous, therefore we need m_pIdToState
    * m_SimBodyStates[bodyNr=3] does not work, m_SimBodyStates[3] might belong to body with id (group:1, bodyNr: 4) and not (group: 1, bodyNr: 3)
    * if it is true however we can directly access m_SimBodyStates[bodyNr]
    */
    std::unordered_map<RigidBodyIdType, RigidBodyState*> m_pIdToState;
    bool m_randomAccess;
};


namespace Interpolate{
    template<typename PREC>
    void lerp( const RigidBodyState & A, const RigidBodyState & B, RigidBodyState & X, PREC factor) {
        X.m_q = A.m_q + factor*(B.m_q - A.m_q);
        X.m_u = A.m_u + factor*(B.m_u - A.m_u);
    };

    template<typename PREC>
    void lerp( const DynamicsStateBase & A, const DynamicsStateBase & B, DynamicsStateBase & X, PREC factor) {
        ASSERTMSG(A.m_SimBodyStates.size() == B.m_SimBodyStates.size() &&  B.m_SimBodyStates.size() == X.m_SimBodyStates.size() ,"Wrong number of bodies!");
        X.m_t = X.m_t = A.m_t + factor*(B.m_t - A.m_t);
        for(int i=0; i<A.m_SimBodyStates.size(); i++){
            lerp(A.m_SimBodyStates[i],B.m_SimBodyStates[i],X.m_SimBodyStates[i],factor);
        }
    };
};



#endif
