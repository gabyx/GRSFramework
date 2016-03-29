// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_states_simulationManager_SimulationManagerBase_hpp
#define GRSF_states_simulationManager_SimulationManagerBase_hpp


#include <boost/thread.hpp>
#include <memory>

#include "GRSF/common/AssertionDebug.hpp"
#include <boost/filesystem.hpp>

#include "GRSF/common/CPUTimer.hpp"

class SimulationManagerBase
{
public:
	SimulationManagerBase();

	~SimulationManagerBase();


	double  getTimelineSimulation();
	void    resetTimelineSimulation();
	void    stopTimelineSimulation();

  // Used by vis thread
  void setThreadToBeStopped(bool stop);   /**	\brief Sets the variable in the vis thread to make the sim thread cancel. */
  bool isSimThreadRunning();              /**	\brief Checks in the vis thread if the thread is still running. */

  enum Threads{ NONE = 0 , REALTIME = 1<<0 ,RECORD = 1<<1, ALL = REALTIME | RECORD  } ;
  enum Threads m_eSimThreadRunning;

  double getTimeScale();
  bool isSimulationPaused();
  // Information to visualize by GUI
  void getIterationTime(double & averageIterationTime, double & maxIterationTime);
  void setIterationTime(double averageIterationTime,   double maxIterationTime);

  unsigned int getNumberOfContacts();
  void         setNumberOfContacts(unsigned int nContacts);

  virtual void setup(boost::filesystem::path sceneFilePath) = 0;

protected:


  boost::thread*	m_pThread;

  boost::mutex m_mutexIterationtime;
	double m_averageIterationTime;
  double m_maxIterationTime;

  boost::mutex m_nCurrentContacts_mutex;
  unsigned int m_nCurrentContacts;

  boost::mutex m_bThreadToBeStopped_mutex;
  bool m_bThreadToBeStopped;

  boost::mutex m_bSimThreadRunning_mutex;
  bool m_bSimThreadRunning;


  // Used by sim thread
  bool isSimThreadToBeStopped();          /**	\brief Checks if the sim thread should be stopped. */
  void setSimThreadRunning(bool value);   /**	\brief Sets the variable which is used in the vis thread to check if the thread is still alive. */

  boost::mutex	m_mutexTimelineSimulation;
  std::shared_ptr<CPUTimer>		m_pTimelineSimulation;
  double m_lastTime; // This is used to stop the timer, set m_lastTime, increase/decrease timeScale, startTimer again

  double m_timeScale;
  double m_timeScaleSave;

  std::vector<double> m_timeScaleList;
  int m_timeScaleListIdx;
  void addToTimeScale(double step);

  void togglePauseSimulation();
  bool m_bPauseEnabled;

  boost::barrier m_barrier_start; // waits for 2 threads! // if needed,

};
//=========================================================

#endif	// SIMULATIONMANAGERBASE_HPP
