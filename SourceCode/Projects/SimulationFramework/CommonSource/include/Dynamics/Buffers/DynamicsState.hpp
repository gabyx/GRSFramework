#ifndef DYNAMICS_STATE_HPP
#define DYNAMICS_STATE_HPP

#include <vector>
#include <unordered_map>

#include "TypeDefs.hpp"
#include "AssertionDebug.hpp"

#include "RigidBodyState.hpp"

//Prototype
class DynamicsStateBase;

namespace Interpolate {
    template<typename PREC>
    void lerp( const DynamicsStateBase & A, const DynamicsStateBase & B, DynamicsStateBase & X, PREC factor);
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
    //RigidBodyStateListType  m_AniBodyStates; ///< A vector comprising of all rigid body states of the system for animated objects.
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

    DynamicsState & operator=(const DynamicsState& state) = default;

    void reset(){
      m_StateType = DynamicsState::NONE;
      m_t = 0.0;
    }

    RigidBodyStateListType::size_type getNSimBodies() const {return m_SimBodyStates.size();}

    template<bool resetState, typename TRigidBodyCont >
    inline void initSimStates(const TRigidBodyCont & cont){
        m_randomAccess = true; m_startIdx = 0;
        m_t = 0.0;
        m_StateType = NONE;

        unsigned int nSimBodies = std::distance(cont.begin(),cont.end());
        ASSERTMSG(nSimBodies, "nSimBodies == 0");
        if(resetState){
            m_SimBodyStates.assign(nSimBodies,RigidBodyState());
        }else{
            m_SimBodyStates.resize(nSimBodies);
        }

        //set startidx
        auto itEnd = cont.end();
        //set startidx
        auto it = cont.begin();
        m_startIdx = RigidBodyId::getBodyNr(*it);
        auto sIt = m_SimBodyStates.begin();
        for(; it!= itEnd;++it){
            // Check for continuity in ids
            auto id = (*it)->m_id;
            if( m_randomAccess && std::next(it)!=itEnd && (*std::next(it))->m_id - (*it)->m_id != 1 ){
                m_randomAccess=false;
            }
            sIt->m_id = id;
            m_pIdToState.insert(std::make_pair(id,&(*sIt)));
        }
        std::cout << "Random Access :" << m_randomAccess << std::endl;
    }

    /** Access the SimState for a given Id*/
    inline RigidBodyState * getSimState(const RigidBodyIdType & id){
        if(m_randomAccess){
            auto bodyNr = RigidBodyId::getBodyNr(id)- m_startIdx;
            if( bodyNr < m_SimBodyStates.size() && bodyNr>=0){
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

    template< bool sequenceMatch = false, typename TRigidBodyStatesCont>
    void applyBodyStates(const TRigidBodyStatesCont & states) {



        // If both sequence (ids of states and internal mSimBodyStates are in sequenc, we can just simply iterate over all states and apply them
        if(sequenceMatch){
            ASSERTMSG(states.size() == m_SimBodyStates.size() , "states container has size: "
            << states.size() << "instead of " << m_SimBodyStates.size())
            auto s = states.begin();
            for(auto & state : m_SimBodyStates){
                state = s->second;
                ++s;
            }
        }else{
            // If sequence does not match, either do the random access version or the no random access version
            // Fill in the initial values
            if(m_randomAccess){ // fast if we have random access!
                auto itEnd = states.end();
                for(auto it = states.begin(); it!=itEnd ; ++it) {
                    unsigned int bodyNr = RigidBodyId::getBodyNr(it->first) - m_startIdx;
                    //WARNINGMSG( (bodyNr < m_SimBodyStates.size() && bodyNr>=0) , "body nr: " << bodyNr << " out of bound for DynamicState!");
                    if( bodyNr < m_SimBodyStates.size() && bodyNr>=0 ){
                        m_SimBodyStates[bodyNr] =  it->second;
                    }
                }
            }else{
                auto itEnd = states.end();
                for(auto it = states.begin(); it!=itEnd ; ++it) {
                    auto id = m_pIdToState.find(it->first); // find id
                    if(id!=m_pIdToState.end()){
                        *id->second = it->second;
                    }
                }
            }
        }
    }

    template< bool sequenceMatch = false, typename TRigidBodyCont>
    inline void applyBodies(const TRigidBodyCont & bodies) {

        ASSERTMSG(m_SimBodyStates.size() == bodies.size(), "Wrong Size" << m_SimBodyStates.size() <<"!="<< bodies.size()<<std::endl );


        if(sequenceMatch){
            auto stateIt = m_SimBodyStates.begin();
            auto itEnd = bodies.end();
            for(auto bodyIt = bodies.begin(); bodyIt != itEnd ; ++bodyIt) {
                stateIt->applyBody(*bodyIt);
                stateIt++;
            }
        }else{

            if(m_randomAccess){
                auto itEnd = bodies.end();
                for(auto it = bodies.begin(); it!=itEnd ; ++it) {
                    unsigned int bodyNr = RigidBodyId::getBodyNr((*it)->m_id) - m_startIdx;
                    //WARNINGMSG( (bodyNr < m_SimBodyStates.size() && bodyNr>=0) , "body nr: " << bodyNr << " out of bound for DynamicState!");
                    if( bodyNr < m_SimBodyStates.size() && bodyNr>=0 ){
                        m_SimBodyStates[bodyNr].applyBody(*it);
                    }
                }
            }else{
                auto itEnd = bodies.end();
                for(auto bodyIt = bodies.begin(); bodyIt!=itEnd ; ++bodyIt) {
                    auto it = m_pIdToState.find((*bodyIt)->m_id); // find id
                    if(it!=m_pIdToState.end()){
                        it->second->applyBody(*bodyIt);
                    }
                }
            }

        }
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
    unsigned int m_startIdx; // needed to acces state with operator[z=3] --> 3-m_startIdx = 0 idx
};


namespace Interpolate{
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
