#ifndef GMSF_Dynamics_Inclusion_InclusionSolverSettings_hpp
#define GMSF_Dynamics_Inclusion_InclusionSolverSettings_hpp

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
      m_RStrategy = RSTRATEGY_MAX;
      m_MinIter = 0;
      m_MaxIter = 5000;
      m_AbsTol = 1E-7;
      m_RelTol = 1E-7;
      m_eMethod = SOR_CONTACT;

      m_useDriftCorrectionGap = false;
      m_driftCorrectionGapAlpha = 0.2;

      m_bUseGPU = false;
      m_UseGPUDeviceId = 0;
      m_bIsFiniteCheck = false;

      m_eConvergenceMethod = InVelocity;
      m_bComputeResidual = false;

    }


    PREC m_deltaT;

    PREC m_alphaJORProx;
    PREC m_alphaSORProx;
    enum RMatrixStrategy{RSTRATEGY_MAX, RSTRATEGY_SUM,RSTRATEGY_SUM2} m_RStrategy;
    unsigned int m_MaxIter;
    unsigned int m_MinIter;
    PREC m_AbsTol;
    PREC m_RelTol;

    /**
    *  SOR_CONTACT (project contacts consecutively)
    *  SOR_FULL ( normal direction first, vel. update, then tangential, vel.update, over each contact)
    *  SOR_NORMAL_TANGENTIAL (for all contacts  first normal , then for all contacts tangential (iteratively solve two convex optimization problems) )
    */
    enum Method{SOR_CONTACT, SOR_FULL, SOR_NORMAL_TANGENTIAL, JOR} m_eMethod;

    /**             X=AC : AlartCurnier for UCF Contacts (normal and tangential normal cones),
    *               X=DS : De Saxe for UCF Contacts (combined normal cone) */
    enum SubMethodUCF{UCF_DS,UCF_AC} m_eSubMethodUCF;

    /** Drift Correction by adding deltaGap/(deltaT/2) to the normal inclusion
    *   This should only be used with eps_n = 0! otherwise impact law is destroyed with MoreauTimeStepper
    */
    bool m_useDriftCorrectionGap;
    PREC m_driftCorrectionGapAlpha;


    enum Convergence {InLambda,InVelocity, InVelocityLocal, InEnergyVelocity,InEnergyLocalMix} m_eConvergenceMethod;
    bool m_bComputeResidual; ///< If true convergence check is done for all contacts/bodies, no break in the loop
    bool  m_bUseGPU;
    int m_UseGPUDeviceId;
    bool m_bIsFiniteCheck;

};



#endif
