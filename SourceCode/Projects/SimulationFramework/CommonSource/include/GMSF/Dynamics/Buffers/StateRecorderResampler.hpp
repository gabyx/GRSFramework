#ifndef GMSF_Dynamics_Buffers_StateRecorderResampler_hpp
#define GMSF_Dynamics_Buffers_StateRecorderResampler_hpp

#include "GMSF/Common/TypeDefs.hpp"
#include "GMSF/Common/LogDefines.hpp"

#include <vector>

#include "GMSF/Singeltons/Contexts/RenderContext.hpp"

#include "GMSF/Dynamics/Buffers/StateRecorder.hpp"

#include <boost/thread.hpp>
#include <boost/filesystem.hpp>

#include "GMSF/Dynamics/Buffers/DynamicsState.hpp"

/**
* @brief only used in sim thread to resample and drop a new sim file if the option is selected in playback! Not mutex locks or something else!
*/
class StateRecorderResampler: public StateRecorder {
public:

   DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW


    StateRecorderResampler(const unsigned int nSimBodies):
      StateRecorder(nSimBodies)
   {
      m_pStateArray.assign(3, DynamicsState());

      reset();
      m_fps = 25; // Standart value
      m_startTime = 0;
      m_endTime = 1000000;
   };

   ~StateRecorderResampler(){};

   void tryToWrite( const DynamicsState* state, bool bInterpolate){
      std::stringstream logstream;
       /*
       LOG(m_pStateRecorderLog, "Current resample time: " << m_currentResampleTime << "/ "<< m_startTime << " /" << m_endTime;);
       */
      if(m_currentResampleTime <= m_endTime){
         *m_pNextState = *state;
         if(state->m_t >= m_currentResampleTime){
            // Check to interpolate...
            double diffTime = m_pNextState->m_t - m_pPrevState->m_t;
            if(diffTime != 0 && bInterpolate){
               double factor = (m_currentResampleTime - m_pPrevState->m_t)/(diffTime);
               Interpolate::lerp(*m_pPrevState, *m_pNextState, *m_pLerpState,factor);
               logstream << ", Interpolate Factor: " <<factor;
               this->write(m_pLerpState);
               logstream << "StateRecorderResampler:: Writen interpolated State: " << m_pLerpState->m_t << " / " << "currentTime: "<< m_currentResampleTime;
            }else{
               this->write(m_pNextState);
               logstream << "StateRecorderResampler:: Writen State: " << m_pNextState->m_t << " / " << "currentTime: "<< m_currentResampleTime;
            }

            m_currentResampleTime += 1.0/m_fps; //Move resample time ahead! for next drop!
            this->m_pSimulationLog->logMessage(logstream);
         }
         std::swap(m_pPrevState, m_pNextState);
      }
   }

   void setFPS(double fps){
      m_fps = fps;
   }

   void setTimeRange(double startTime, double endTime){
      m_startTime = startTime;
      m_currentResampleTime = m_startTime;
      m_endTime = endTime;
   }

   void reset(){
      m_nStatesDropped = 0;
      m_currentResampleTime = m_startTime;
      m_bFirstInsert = true;

      m_pPrevState = &m_pStateArray[0];
      m_pNextState = &m_pStateArray[1];
      m_pLerpState = &m_pStateArray[2];
   }

   void setFolderPath(boost::filesystem::path folderPath){
      m_folderPath = folderPath;
   }

private:
   std::vector< DynamicsState > m_pStateArray;
   DynamicsState * m_pPrevState;
   DynamicsState * m_pNextState;
   DynamicsState * m_pLerpState;
   bool m_bFirstInsert;

   boost::filesystem::path m_folderPath;

   double m_currentResampleTime;
   double m_fps; //[frames/second]
   double m_startTime;
   double m_endTime;

   unsigned int m_nStatesDropped;
};


#endif
