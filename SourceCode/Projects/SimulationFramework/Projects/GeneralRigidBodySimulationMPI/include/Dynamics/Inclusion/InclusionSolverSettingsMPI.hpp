#ifndef InclusionSolverSettings_hpp
#define InclusionSolverSettings_hpp

#include "TypeDefs.hpp"


#define USE_PERCUSSION_POOL 0

/**
* @ingroup Inclusion
* @brief The inclusion solver settings.
*/
struct InclusionSolverSettings
{
    DEFINE_LAYOUT_CONFIG_TYPES

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
      m_eConvergenceMethod = InVelocity;
      m_splitNodeUpdateRatio = 1;
      m_convergenceCheckRatio = 1;
    }


    PREC m_deltaT;
    PREC m_alphaJORProx;
    PREC m_alphaSORProx;

    unsigned int m_MaxIter;
    unsigned int m_MinIter;

    unsigned int m_splitNodeUpdateRatio;        ///< Local iterations per remote billateral constraints updates (splitNodes)
    unsigned int m_convergenceCheckRatio;      ///< Billatreal constraints updates (splitNodes)  per convergence checks

    enum Method{ SOR, JOR} m_eMethod;
    enum Convergence {InLambda,InVelocity, InVelocityLocal, InEnergyVelocity,InEnergyLocalMix} m_eConvergenceMethod;
    PREC m_AbsTol;
    PREC m_RelTol;

    bool  m_bUseGPU;
    int m_UseGPUDeviceId;
    bool m_bIsFiniteCheck;

};



#endif
