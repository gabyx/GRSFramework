#ifndef GRSF_Dynamics_General_TimeStepperSettings_hpp
#define GRSF_Dynamics_General_TimeStepperSettings_hpp

#include "GRSF/Common/TypeDefs.hpp"

#include <boost/filesystem.hpp>

/**
* @ingroup DynamicsGeneral
* @brief The time stepper settings.
*/
struct TimeStepperSettings
{
  DEFINE_LAYOUT_CONFIG_TYPES
  TimeStepperSettings(){
      //standart values
      m_deltaT = 0.001;
      m_startTime = 0;
      m_endTime = 10;
      m_simStateReferenceFile = boost::filesystem::path(); ///< The reference file which is used to simulate, either continue or use_states!
      m_simDataReferenceFile = boost::filesystem::path(); // No implemented yet
      m_eSimulateFromReference = NONE;
  }

  PREC	m_deltaT;
  PREC	m_startTime;
  PREC	m_endTime;


  enum {NONE,CONTINUE,USE_STATES} m_eSimulateFromReference;
  boost::filesystem::path m_simStateReferenceFile;
  boost::filesystem::path m_simDataReferenceFile;
};

#endif