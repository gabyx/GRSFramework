#ifndef PlaybackLoader_hpp
#define PlaybackLoader_hpp


#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>

#include "DynamicsState.hpp"
#include "MultiBodySimFile.hpp"

#include "SimpleLogger.hpp"

/**
* @ingroup SimulationManager
* @brief Playback Loader which reads a MultiBodySimFile in a thread and load it into a buffer.
*/
template< typename TLayoutConfig , typename TStatePool>
class PlaybackLoader{
public:
  DEFINE_LAYOUT_CONFIG_TYPES_OF(TLayoutConfig)

  PlaybackLoader(const unsigned int nSimBodies, boost::shared_ptr<TStatePool>	pStatePool);
  ~PlaybackLoader();


  void setThreadToBeStopped(bool stop);
  bool isLoaderThreadRunning();

  void setReadFullState(bool value);

  void startLoaderThread();
  void stopLoaderThread();

  boost::barrier m_barrier_start;

  const unsigned int m_nDofu, m_nDofq; // These are the global dimensions of q and u
  const unsigned int m_nDofuObj, m_nDofqObj, m_nSimBodies; // These are the dimensions for one Obj

  MultiBodySimFile<TLayoutConfig> m_BinarySimFile;

private:
  Logging::Log * m_pThreadLog; /**< This log is set to the thread log which calls this loader thread.  */

  boost::thread*	m_pThreadDynamics;
  void initLoaderThread();
  void runLoaderThread();

  void reset();
  bool loadFile();
  void unloadFile();

  boost::mutex m_bLoaderThreadRunning_mutex;
  bool m_bLoaderThreadRunning;
  void setLoaderThreadRunning(bool value);

  boost::mutex m_bThreadToBeStopped_mutex;
  bool m_bThreadToBeStopped;
  bool isLoaderThreadToBeStopped();

  bool m_bReadFullState;

  boost::shared_ptr< DynamicsState<TLayoutConfig> > m_state; /**<   This is the actual loader state pointer ;*/
  boost::shared_ptr<TStatePool>	m_pStatePool;
};

// Implementation


#include "FileManager.hpp"

template< typename TLayoutConfig , typename TStatePool>
PlaybackLoader<TLayoutConfig, TStatePool>::~PlaybackLoader()
{

}
template< typename TLayoutConfig , typename TStatePool>
PlaybackLoader<TLayoutConfig, TStatePool>::PlaybackLoader( const unsigned int nSimBodies, boost::shared_ptr<TStatePool> pStatePool):
m_barrier_start(2),
m_nSimBodies(nSimBodies),
m_nDofqObj(NDOFqObj),
m_nDofuObj(NDOFuObj),
m_nDofq(m_nSimBodies * m_nDofqObj),
m_nDofu(m_nSimBodies * m_nDofuObj)
{
  //Set the Log Output =========================================================================
  m_pThreadLog = new Logging::Log("PlacbackLoaderThreadLog");
  Logging::LogManager::getSingletonPtr()->registerLog(m_pThreadLog);

#if LogToFileLoader == 1
  boost::filesystem::path filePath = GLOBAL_LOG_FOLDER_DIRECTORY;
  filePath /= "PlacbackLoaderThread.log";
  m_pThreadLog->addSink(new Logging::LogSinkFile("PlacbackLoader-File",filePath));
#endif
#if LogToConsoleLoader == 1
  m_pThreadLog->addSink(new Logging::LogSinkCout("PlacbackLoader-Cout"));
#endif

  m_bLoaderThreadRunning = false;
  m_bReadFullState = false;
  m_pStatePool = pStatePool;

}


template< typename TLayoutConfig , typename TStatePool>
void PlaybackLoader<TLayoutConfig, TStatePool>::setReadFullState(bool value){
   m_bReadFullState = value;
}

template< typename TLayoutConfig , typename TStatePool>
void PlaybackLoader<TLayoutConfig, TStatePool>::stopLoaderThread()
{
  if (isLoaderThreadRunning())
  {
      setThreadToBeStopped(true);
      m_pThreadDynamics->join();
      m_pThreadLog->logMessage("PlaybackLoader:: Stop Thread: Loader Thread has been stopped!");
      m_bLoaderThreadRunning = false;

  }else{
    m_pThreadLog->logMessage("PlaybackLoader:: Stop Thread: Loader Thread is not running!");
  }

}
template< typename TLayoutConfig , typename TStatePool>
void PlaybackLoader<TLayoutConfig, TStatePool>::startLoaderThread()
{

  if (!isLoaderThreadRunning())
  {

    setThreadToBeStopped(false);
    //Start Loader Thread===========================================
      initLoaderThread(); // Initialize everything so that we can immediatly start in the loop
      m_pThreadDynamics = new boost::thread( boost::bind(&PlaybackLoader::runLoaderThread, this) );
      m_pThreadLog->logMessage(" PlaybackLoader:: Loader Thread for Playback started ...");

  }else{
    m_pThreadLog->logMessage("PlaybackLoader:: There is already on Loader Thread running!");
  }
}


template< typename TLayoutConfig , typename TStatePool>
void PlaybackLoader<TLayoutConfig, TStatePool>::initLoaderThread()
{

}

template< typename TLayoutConfig , typename TStatePool>
void PlaybackLoader<TLayoutConfig, TStatePool>::runLoaderThread()
{
  static std::stringstream logstream;
  static bool bMovedBuffer;
  enum {FILE_CHECK,MOVE_POINTER, READ_IN,FINALIZE_AND_BREAK,EXIT} current_state;

    m_pThreadLog->logMessage(" PlaybackLoader: LoaderThread entering...");
    setLoaderThreadRunning(true);

    if(loadFile()){

      LOG(m_pThreadLog, << " File loaded: Number of States = " << m_BinarySimFile.getNStates(););

      reset();

      // Fill buffering =====================================================================


      bMovedBuffer = true;
      unsigned int i=0;
      current_state = READ_IN;
      while(!isLoaderThreadToBeStopped())
      {

         if(current_state== FILE_CHECK){
            if(m_BinarySimFile.isGood()){
               current_state = MOVE_POINTER;
            }else{
               // Write end flag to state!
               m_pThreadLog->logMessage("Write endstate");
               m_state->m_StateType = DynamicsState<TLayoutConfig>::ENDSTATE;
               current_state = FINALIZE_AND_BREAK;
            }
         }
         else if(current_state== MOVE_POINTER){
            m_state = m_pStatePool->advanceLoadBuffer(bMovedBuffer);
             if(!bMovedBuffer){
               m_pThreadLog->logMessage("File buffering finished...");
               current_state=FILE_CHECK;
               break;
             }else{
               current_state=READ_IN;
             }
         }else if(current_state== READ_IN){
           i++;
           m_BinarySimFile >> m_state.get();
            if(i % 20==0){
               LOG(m_pThreadLog, <<  "File loader buffering state: " << m_state->m_t <<"...";);
           }
           current_state = FILE_CHECK;
         }
         else if(current_state== FINALIZE_AND_BREAK){
            //Move pointer once more! To make this state avalibale to the vis pointer!
            break;
         }

      }
      // =========================================================================================


      // wait for caller thread;
      m_barrier_start.wait();

      while(!isLoaderThreadToBeStopped() && current_state != EXIT)
      {

         if(current_state== FILE_CHECK){
            if(m_BinarySimFile.isGood()){
               current_state = MOVE_POINTER;
            }else{
               // Write end flag to state!
               LOG(m_pThreadLog, <<  "Write endstate to m_state m_t:" << m_state->m_t <<endl;);
               m_state->m_StateType = DynamicsState<TLayoutConfig>::ENDSTATE;
               current_state = FINALIZE_AND_BREAK;
            }
         }
         else if(current_state== MOVE_POINTER){
            m_state = m_pStatePool->advanceLoadBuffer(bMovedBuffer);
            if(bMovedBuffer){
               current_state = READ_IN;
            }
         }else if(current_state== READ_IN){
           m_BinarySimFile >> m_state.get();
               /*
               LOG(m_pThreadLog, <<  "Loaded m_t:" << m_state->m_t <<endl;);
                */
           current_state = FILE_CHECK;
         }else if(current_state== FINALIZE_AND_BREAK){
            //Move pointer once more! To make this state avalibale to the vis pointer!
            m_state = m_pStatePool->advanceLoadBuffer(bMovedBuffer);
            if(bMovedBuffer){
               m_pThreadLog->logMessage("End of file reached...");
               current_state=EXIT;
               break;
            }
         }

      }

      // Close file
      unloadFile();

    }else{
      m_barrier_start.wait();
      m_pThreadLog->logMessage(" PlaybackLoader: There was a file error! ");
    }


     m_pThreadLog->logMessage(" PlaybackLoader: LoaderThread leaving...");
     setLoaderThreadRunning(false);
}

template< typename TLayoutConfig , typename TStatePool>
void PlaybackLoader<TLayoutConfig, TStatePool>::reset()
{
  m_state = m_pStatePool->getLoadBuffer();
}


template< typename TLayoutConfig , typename TStatePool>
void PlaybackLoader<TLayoutConfig, TStatePool>::setLoaderThreadRunning( bool value )
{
  boost::mutex::scoped_lock l(m_bLoaderThreadRunning_mutex);
  m_bLoaderThreadRunning = value;
}
template< typename TLayoutConfig , typename TStatePool>
bool PlaybackLoader<TLayoutConfig, TStatePool>::isLoaderThreadRunning()
{
  boost::mutex::scoped_lock l(m_bLoaderThreadRunning_mutex);
  return m_bLoaderThreadRunning;
}

template< typename TLayoutConfig , typename TStatePool>
void PlaybackLoader<TLayoutConfig, TStatePool>::setThreadToBeStopped( bool stop )
{
  boost::mutex::scoped_lock l(m_bThreadToBeStopped_mutex);
  m_bThreadToBeStopped = stop;
}

template< typename TLayoutConfig , typename TStatePool>
bool PlaybackLoader<TLayoutConfig, TStatePool>::isLoaderThreadToBeStopped()
{
  boost::mutex::scoped_lock l(m_bThreadToBeStopped_mutex);
  return m_bThreadToBeStopped;
}



template< typename TLayoutConfig , typename TStatePool>
bool PlaybackLoader<TLayoutConfig, TStatePool>::loadFile()
{
  using namespace boost::filesystem;

  path file_path = FileManager::getSingletonPtr()->getPathSimFileSelected();
  if(file_path.empty()){
    m_pThreadLog->logMessage("PlaybackLoader:: You have no file selected, please rescan the directory...");
    return false;
  }

  if(boost::filesystem::exists(file_path)){
    if(is_regular_file(file_path)){
      if(!boost::filesystem::is_empty(file_path)){

        // Try to load the file
        if(m_BinarySimFile.openSimFileRead(file_path,m_nSimBodies,m_bReadFullState))
        {
          return true;
        }else{
           std::stringstream error;
           error << "PlaybackLoader:: Could not open file: " << file_path.string();
           error << "File errors: " <<endl<< m_BinarySimFile.getErrorString();
           m_pThreadLog->logMessage(error.str());
        }
      }
      else{
        m_pThreadLog->logMessage("PlaybackLoader:: File Error:  File is empty...");
      }
    }
    else{
     m_pThreadLog->logMessage("PlaybackLoader:: File Error: File is not a regular file...");
    }
  }
  else{
    m_pThreadLog->logMessage("PlaybackLoader:: File Error: File does not exist...");
  }

  return false;
}


template< typename TLayoutConfig , typename TStatePool>
void PlaybackLoader<TLayoutConfig, TStatePool>::unloadFile()
{
  m_BinarySimFile.closeSimFile();
  m_pThreadLog->logMessage("PlaybackLoader:: File closed...");
}

#endif
