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

    };



    PREC m_multFactor;  ///< This is the factor in how many virtual parts the body is split during the global inlcusion solving process

};

#endif

