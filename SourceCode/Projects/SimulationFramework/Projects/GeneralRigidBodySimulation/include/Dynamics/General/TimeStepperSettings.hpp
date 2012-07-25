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

  TimeStepperSettings(){
      //standart values
      m_deltaT = 0.001;
      m_endTime = 10;
      m_simStateReferenceFile = boost::filesystem::path();
      m_simDataReferenceFile = boost::filesystem::path();
      m_eSimulateFromReference = NONE;
  }

  PREC	m_deltaT;
  PREC	m_endTime;


  enum {NONE,CONTINUE,USE_STATES} m_eSimulateFromReference;
  boost::filesystem::path m_simStateReferenceFile;
  boost::filesystem::path m_simDataReferenceFile;
};

#endif
