﻿#ifndef InclusionSolverSettings_hpp
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
      m_eMethod = SOR_CONTACT;
      m_bUseGPU = false;
      m_UseGPUDeviceId = 0;
      m_bIsFiniteCheck = false;

      m_eConvergenceMethod = InVelocity;
      m_bComputeResidual = false; ///< If true convergence check is done for all contacts/bodies, no break in the loop

      //Experimental
      //m_dinv = 0; //Inverse Damper Coefficient which acts on the diagonal of G,

    }


    PREC m_deltaT;
    PREC m_alphaJORProx;
    PREC m_alphaSORProx;
    unsigned int m_MaxIter;
    unsigned int m_MinIter;
    PREC m_AbsTol;
    PREC m_RelTol;
    /**
    *  SOR_CONTACT (project contacts consecutively), SOR_FULL (normal direction first, then tangential)
    */
    enum Method{SOR_CONTACT, SOR_FULL, JOR} m_eMethod;
    enum Convergence {InLambda,InVelocity, InVelocityLocal, InEnergyVelocity,InEnergyLocalMix} m_eConvergenceMethod;
    bool m_bComputeResidual;
    bool  m_bUseGPU;
    int m_UseGPUDeviceId;
    bool m_bIsFiniteCheck;

};



#endif
