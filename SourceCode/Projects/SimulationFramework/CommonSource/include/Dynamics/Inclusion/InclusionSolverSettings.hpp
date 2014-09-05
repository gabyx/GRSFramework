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
      m_eMethod = SOR_CONTACT_AC;
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
    *  SOR_CONTACT_X (project contacts consecutively,
    *               X=AC : AlartCurnier for UCF Contacts (normal and tangential normal cones),
    *               X=DS : De Saxe for UCF Contacts (combined normal cone)
    *  SOR_FULL (AlarCurnier for UCF Contacts, normal direction first, vel. update, then tangential, vel.update, over each contact)
    *  SOR_NORMAL_TANGENTIAL (for all contacts  first normal , then for all contacts tangential (iteratively solve two convex optimization problems) )
    */
    enum Method{SOR_CONTACT_AC, SOR_CONTACT_DS, SOR_FULL, SOR_NORMAL_TANGENTIAL, JOR} m_eMethod;
    enum Convergence {InLambda,InVelocity, InVelocityLocal, InEnergyVelocity,InEnergyLocalMix} m_eConvergenceMethod;
    bool m_bComputeResidual;
    bool  m_bUseGPU;
    int m_UseGPUDeviceId;
    bool m_bIsFiniteCheck;

};



#endif
