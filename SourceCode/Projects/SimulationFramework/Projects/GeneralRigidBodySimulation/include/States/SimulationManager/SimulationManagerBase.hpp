#ifndef SimulationManagerBase_hpp
#define SimulationManagerBase_hpp


#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <OGRE/Ogre.h>

#include "AssertionDebug.hpp"

#include <boost/timer/timer.hpp>

class SimulationManagerBase
{
public:
	SimulationManagerBase();

	~SimulationManagerBase();


	double getTimelineSimulation();
	void resetTimelineSimulation();

  // Used by vis thread
  void setThreadToBeStopped(bool stop);   /**	\brief Sets the variable in the vis thread to make the sim thread cancel. */
  bool isSimThreadRunning();              /**	\brief Checks in the vis thread if the thread is still running. */

  enum Threads{ NONE = 0 , REALTIME = 1<<0 ,RECORD = 1<<1, ALL = REALTIME | RECORD  } ;
  enum Threads m_eSimThreadRunning;

  double getTimeScale();
  // Information to visualize by GUI
  void getIterationTime(double & averageIterationTime, double & maxIterationTime);
  void setIterationTime(double averageIterationTime,   double maxIterationTime);
  unsigned int getNumberOfContacts();
  void setNumberOfContacts(unsigned int & nContacts);

  virtual void setup() = 0;
  virtual void updateScene(double timeSinceLastFrame) = 0;

  virtual double getSimulationTime(){return 0;};

  virtual void startSimThread(Threads threadToStart)=0;
  virtual void stopSimThread(Threads threadToStop, bool force_stop)=0;


protected:

  Ogre::Log*	m_pThreadLog;
  Ogre::Log*  m_pAppLog;

  boost::thread*	m_pThread;

   virtual void initBeforeThreads(){};

   virtual void initSimThread(){};
	virtual void threadRunSimulation()=0;
  virtual void initRecordThread(){};
  virtual void threadRunRecord()=0;




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

  boost::mutex		m_mutexTimelineSimulation;
  boost::shared_ptr<boost::timer::cpu_timer>		m_pTimelineSimulation;
  double m_lastTime; // This is used to stop the timer, set m_lastTime, increase/decrease timeScale, startTimer again

  double	m_timeScale;
  std::vector<double> m_timeScaleList;
  int m_timeScaleListIdx;
  void addToTimeScale(double step);

  boost::barrier m_barrier_start; // waits for 2 threads! // if needed,

};
//=========================================================

#endif	// SIMULATIONMANAGERBASE_HPP
