#ifndef TimeStepperSettings_hpp
#define TimeStepperSettings_hpp

#include "TypeDefs.hpp"

#include <boost/filesystem.hpp>

/**
* @ingroup DynamicsGeneral
* @brief The time stepper settings.
*/
template<typename TLayoutConfig>
struct TimeStepperSettings
{
  DEFINE_LAYOUT_CONFIG_TYPES_OF(TLayoutConfig)
  PREC	m_deltaT;
  PREC	m_endTime;

  enum {NONE,CONTINUE,USE_STATES} m_eSimulateFromReference;
  boost::filesystem::path m_stateReferenceFile;
};

#endif