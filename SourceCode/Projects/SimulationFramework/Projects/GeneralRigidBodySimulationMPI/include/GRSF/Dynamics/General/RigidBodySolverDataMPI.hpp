#ifndef GRSF_Dynamics_General_RigidBodySolverDataMPI_hpp
#define GRSF_Dynamics_General_RigidBodySolverDataMPI_hpp

#include "GRSF/Common/TypeDefs.hpp"

#include "GRSF/Dynamics/General/RigidBodySolverData.hpp"

/** Class with  Data Structure for the Solver! */
class RigidBodySolverDataCONoGMPI : public RigidBodySolverDataCONoG {

    public:
    DEFINE_LAYOUT_CONFIG_TYPES
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    RigidBodySolverDataCONoGMPI(): RigidBodySolverDataCONoG(){
        m_multiplicityWeight = 1.0;
        m_multiplicity = 1; // If one this body is not split, if >1 then it is split
    };

    /** Dont change these values directly, do this over the two functions
        RigidBodyFunctions::changeBodyToSplitWeighting and
        RigidBodyFunctions::changeBodyToNormalWeighting
    */
    PREC m_multiplicityWeight;  ///< This is the actual weight factor which is used by this process, to scale the mass matrix and the h vector!
    unsigned int m_multiplicity;  ///< This is the factor in how many virtual parts the body is split during the global inlcusion solving process

};

#endif

