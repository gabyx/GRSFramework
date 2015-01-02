#ifndef GRSF_Dynamics_General_RigidBodySolverData_hpp
#define GRSF_Dynamics_General_RigidBodySolverData_hpp

#include "GRSF/Common/TypeDefs.hpp"

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
        m_uBegin.setZero();
        m_uBuffer.m_front.setZero();
        m_uBuffer.m_back.setZero();
    };



    bool m_bInContactGraph; ///< Flag which determines if this body is in the contact graph!

    VectorUBody m_uBegin; ///< Velocity at beginning of timestep, important to save this for timestep update in timestepper!

    /** Pointers into the right Front BackBuffer for the velocity which get iteratet in the InclusionSolverCONoG
    * The back buffer is the velocity before the prox iteration (over all nodes)
    * The front buffer is the velocity which is used to during ONE prox iteration
    */
    FrontBackBuffer<VectorUBody,FrontBackBufferPtrType::NoPtr, FrontBackBufferMode::NoConst> m_uBuffer;


    void swapBuffer(){
        m_uBegin = m_uBuffer.m_front; // Set Begin to the new front velocity
        m_uBuffer.m_front.swap(m_uBuffer.m_back); // swap buffer
    }

    void reset(){
        m_uBuffer.m_front.setZero();
        m_bInContactGraph = false;
    };

};
#endif
