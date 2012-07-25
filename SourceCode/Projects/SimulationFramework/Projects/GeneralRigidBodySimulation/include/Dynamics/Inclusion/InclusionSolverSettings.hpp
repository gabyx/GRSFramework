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

    InclusionSolverSettings()
    {
      //Standard values
      m_deltaT = 0.001;
      m_alphaJORProx = 0.5;
      m_alphaSORProx = 1.2;
      m_MaxIter = 5000;
      m_AbsTol = 1E-7;
      m_RelTol = 1E-7;
      m_eMethod = SOR;
      m_bUseGPU = false;
      m_UseGPUDeviceId = 0;
      m_bIsFiniteCheck = false;
    }


    PREC m_deltaT;
    PREC m_alphaJORProx;
    PREC m_alphaSORProx;
    unsigned int m_MaxIter;
    unsigned int m_MinIter;
    PREC m_AbsTol;
    PREC m_RelTol;
    enum Method{ SOR, JOR} m_eMethod;
    enum Convergence {InLambda,InVelocity, InVelocityLocal, InEnergyVelocity,InEnergyLocalMix} m_eConvergenceMethod;
    bool  m_bUseGPU;
    int m_UseGPUDeviceId;
    bool m_bIsFiniteCheck;

};



#endif
