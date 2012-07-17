#ifndef InclusionSolverSettings_hpp
#define InclusionSolverSettings_hpp

#include "TypeDefs.hpp"


#define USE_PERCUSSION_POOL 0

/**
* @ingroup Inclusion
* @brief The inclusion solver settings.
*/
template<typename TLayoutConfig>
struct InclusionSolverSettings
{
    DEFINE_LAYOUT_CONFIG_TYPES_OF(TLayoutConfig)

    PREC m_deltaT;
    PREC m_alphaJORProx;
    PREC m_alphaSORProx;
    unsigned int m_MaxIter;
    unsigned int m_MinIter;
    PREC m_AbsTol;
    PREC m_RelTol;
    enum Method{ SOR, JOR} m_eMethod;
    enum Convergence {InLambda,InVelocity, InVelocityLocal} m_eConvergenceMethod;
    bool  m_bUseGPU;
    int m_UseGPUDeviceId;
    bool m_bIsFiniteCheck;

};



#endif
