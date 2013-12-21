#ifndef RigidBodySolverDataMPI_hpp
#define RigidBodySolverDataMPI_hpp

#include "TypeDefs.hpp"

#include "RigidBodySolverData.hpp"

/** Class with  Data Structure for the Solver! */
class RigidBodySolverDataCONoGMPI : public RigidBodySolverDataCONoG {

    public:
    DEFINE_LAYOUT_CONFIG_TYPES
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    RigidBodySolverDataCONoGMPI(): RigidBodySolverDataCONoG(){
        m_multiplicityWeight = 1.0;
        m_multFactor = 1; // If one this body is not split, if >1 then it is split
    };

    PREC m_multiplicityWeight;  ///< This is the actual weight factor which is used by this process, to scale the mass matrix and the h vector!
    unsigned int m_multFactor;  ///< This is the factor in how many virtual parts the body is split during the global inlcusion solving process

};

#endif

