#ifndef InclusionSolverSettingsMPI_hpp
#define InclusionSolverSettingsMPI_hpp

#include "TypeDefs.hpp"


#define USE_PERCUSSION_POOL 0

/**
* @ingroup Inclusion
* @brief The inclusion solver settings.
*/
struct InclusionSolverSettingsMPI
{
    DEFINE_LAYOUT_CONFIG_TYPES

    InclusionSolverSettingsMPI()
    {
      //Standard values
      m_deltaT = 0.001;
      m_alphaJORProx = 0.5;
      m_alphaSORProx = 1.2;
      m_MaxIter = 5000;
      m_AbsTol = 1E-7;
      m_RelTol = 1E-7;
      m_eMethod = SOR_CONTACT_AC;
      m_bUseGPU = false;
      m_UseGPUDeviceId = 0;
      m_bIsFiniteCheck = false;
      m_eConvergenceMethod = InVelocity;
      m_bComputeResidual = false; ///< If true convergence check is done for all contacts/bodies, no break in the loop (does not work yet)

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

    /**
    *  SOR_CONTACT_X (project contacts consecutively,
    *               X=AC : AlartCurnier for UCF Contacts (normal and tangential normal cones),
    *               X=DS : De Saxe for UCF Contacts (combined normal cone)
    *  SOR_FULL (AlarCurnier for UCF Contacts, normal direction first, vel. update, then tangential, vel.update, over each contact)
    *  SOR_NORMAL_TANGENTIAL (for all contacts  first normal , then for all contacts tangential (iteratively solve two convex optimization problems) )
    */
    enum Method{SOR_CONTACT_AC, SOR_CONTACT_DS, SOR_FULL, SOR_NORMAL_TANGENTIAL, JOR} m_eMethod;
    enum Convergence {InLambda,InVelocity, InVelocityLocal, InEnergyVelocity,InEnergyLocalMix} m_eConvergenceMethod;
    bool m_bComputeResidual;
    PREC m_AbsTol;
    PREC m_RelTol;

    bool  m_bUseGPU;
    int m_UseGPUDeviceId;
    bool m_bIsFiniteCheck;

};



#endif
