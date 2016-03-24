#ifndef GRSF_dynamics_inclusion_InclusionSolverSettingsMPI_hpp
#define GRSF_dynamics_inclusion_InclusionSolverSettingsMPI_hpp

#include "GRSF/common/TypeDefs.hpp"
#include "GRSF/dynamics/inclusion/InclusionSolverSettings.hpp"

#define USE_PERCUSSION_POOL 0

/**
* @ingroup Inclusion
* @brief The inclusion solver settings.
*/
struct InclusionSolverSettingsMPI : InclusionSolverSettings
{
    DEFINE_LAYOUT_CONFIG_TYPES

    InclusionSolverSettingsMPI() : InclusionSolverSettings()
    {
      m_splitNodeUpdateRatio = 1;
      m_convergenceCheckRatio = 1;
    }

    /** Contact Graph reserve space for split nodes**/
    unsigned int m_reserveSplitNodes = 2000;

    unsigned int m_splitNodeUpdateRatio;       ///< Local iterations per remote billateral constraints updates (splitNodes)
    unsigned int m_convergenceCheckRatio;      ///< Billatreal constraints updates (splitNodes)  per convergence checks

};



#endif
