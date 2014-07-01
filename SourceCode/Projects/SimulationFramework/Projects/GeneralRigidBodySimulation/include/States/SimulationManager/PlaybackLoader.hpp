#ifndef PlaybackLoader_hpp
#define PlaybackLoader_hpp


#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <memory>

#include "DynamicsState.hpp"
#include "MultiBodySimFile.hpp"

#include "SimpleLogger.hpp"

/**
* @ingroup SimulationManager
* @brief Playback Loader which reads a MultiBodySimFile in a thread and load it into a buffer.
*/
template<typename TStatePool>
class PlaybackLoader{
public:

  DEFINE_LAYOUT_CONFIG_TYPES

  PlaybackLoader(const unsigned int nSimBodies, std::shared_ptr<TStatePool>	pStatePool);
  ~PlaybackLoader();


  void setThreadToBeStopped(bool stop);
  bool isLoaderThreadRunning();

  void setReadFullState(bool value);

  void startLoaderThread();
  void stopLoaderThread();

  boost::barrier m_barrier_start;

  const unsigned int m_nSimBodies; // These are the dimensions for one Obj

  MultiBodySimFile m_binarySimFile;

private:
  Logging::Log * m_pThreadLog; /**< This log is set to the thread log which calls this loader thread.  */

  boost::thread*	m_pThreadDynamics;
  void initLoaderThread();
  void runLoaderThread();

  void reset();
  bool loadNextFile();
  void unloadFile();

  boost::mutex m_bLoaderThreadRunning_mutex;
  bool m_bLoaderThreadRunning;
  void setLoaderThreadRunning(bool value);

  boost::mutex m_bThreadToBeStopped_mutex;
  bool m_bThreadToBeStopped;
  bool isLoaderThreadToBeStopped();

  bool m_bReadFullState;

  DynamicsState * m_state; /**<   This is the actual loader state pointer ;*/
  std::shared_ptr<TStatePool>	m_pStatePool;

  std::set<boost::filesystem::path> m_simFileList;
  std::set<boost::filesystem::path>::iterator m_currentFileIt;
  unsigned int m_currentFileIndex;
};

// Implementation


#include "FileManager.hpp"

template<typename TStatePool>
PlaybackLoader<TStatePool>::~PlaybackLoader()
{
  delete m_pThreadLog;
}
template<typename TStatePool>
PlaybackLoader<TStatePool>::PlaybackLoader( const unsigned int nSimBodies, std::shared_ptr<TStatePool> pStatePool):
m_barrier_start(2),
m_nSimBodies(nSimBodies)
{
  //Set the Log Output =========================================================================
  m_pThreadLog = new Logging::Log("PlaybackLoaderThreadLog");


  boost::filesystem::path filePath = FileManager::getSingletonPtr()->getLocalDirectoryPath();
        filePath /= GLOBAL_LOG_FOLDER_DIRECTORY;
  filePath /= "PlaybackLoaderThread.log";
#if LogToFileLoader == 1

  m_pThreadLog->addSink(new Logging::LogSinkFile("PlaybackLoader-File",filePath));
#endif
#if LogToConsoleLoader == 1
  m_pThreadLog->addSink(new Logging::LogSinkCout("PlaybackLoader-Cout"));
#endif

  m_bLoaderThreadRunning = false;
  m_bReadFullState = false;
  m_pStatePool = pStatePool;

}


template<typename TStatePool>
void PlaybackLoader<TStatePool>::setReadFullState(bool value){
   m_bReadFullState = value;
}

template<typename TStatePool>
void PlaybackLoader<TStatePool>::stopLoaderThread()
{
  if (isLoaderThreadRunning())
  {
      setThreadToBeStopped(true);
      m_pThreadDynamics->join();
      m_pThreadLog->logMessage("---> PlaybackLoader:: Stop Thread: Loader Thread has been stopped!");
      m_bLoaderThreadRunning = false;

  }else{
    m_pThreadLog->logMessage("---> PlaybackLoader:: Stop Thread: Loader Thread is not running!");
  }

}
template<typename TStatePool>
void PlaybackLoader<TStatePool>::startLoaderThread()
{

  if (!isLoaderThreadRunning())
  {

    setThreadToBeStopped(false);
    //Start Loader Thread===========================================
      initLoaderThread(); // Initialize everything so that we can immediatly start in the loop
      m_pThreadDynamics = new boost::thread( boost::bind(&PlaybackLoader::runLoaderThread, this) );
      m_pThreadLog->logMessage("---> PlaybackLoader:: Loader Thread for Playback started ...");

  }else{
    m_pThreadLog->logMessage("---> PlaybackLoader:: There is already on Loader Thread running!");
  }
}


template<typename TStatePool>
void PlaybackLoader<TStatePool>::initLoaderThread()
{

    // Set the sim file list to the actual current simfolder of the FileManager
    m_simFileList = FileManager::getSingletonPtr()->getPathsSimFilesOfCurrentSimFolder();
    m_currentFileIt = m_simFileList.begin();
    m_currentFileIndex = -1;
}

template<typename TStatePool>
void PlaybackLoader<TStatePool>::runLoaderThread()
{
  static std::stringstream logstream;
  static bool bMovedBuffer;
  enum {LOAD_NEXT_FILE, FILE_CHECK,MOVE_POINTER, READ_IN,FINALIZE_AND_BREAK,EXIT} current_state;

    m_pThreadLog->logMessage("---> PlaybackLoader: LoaderThread entering...");
    setLoaderThreadRunning(true);

    reset();

      // Fill buffer =====================================================================

      bMovedBuffer = true;
      unsigned int i=0;
      current_state = LOAD_NEXT_FILE;
      while(!isLoaderThreadToBeStopped() && current_state != EXIT)
      {
         if(current_state== LOAD_NEXT_FILE ){
            //Load next file
            if(m_currentFileIndex != m_simFileList.size()-1){
                if(!loadNextFile()){
                    current_state = EXIT;
                }else{
                    LOG(m_pThreadLog, "---> SimFile: " << m_currentFileIndex<<"/"<<m_simFileList.size()<<":" << *m_currentFileIt<<" loaded: Number of States = " << m_binarySimFile.getNStates() << std::endl;);
                    current_state = READ_IN;
                }
            }else{
                 // Write end flag to state if that was the last file!
                LOG(m_pThreadLog,  "---> Write endstate to m_state m_t:" << m_state->m_t <<std::endl;);
                m_state->m_StateType = DynamicsState::ENDSTATE;
                current_state = FINALIZE_AND_BREAK;
            }
         }
         else if(current_state== FILE_CHECK){
            //Check if we can read one more state
            if(m_binarySimFile.isGood()){
               current_state = MOVE_POINTER;
            }else{
               current_state = LOAD_NEXT_FILE;
            }
         }
         else if(current_state== MOVE_POINTER){
            // Moves pointer as long as the buffer is no full
            m_state = m_pStatePool->advanceLoadBuffer(bMovedBuffer);
             if(!bMovedBuffer){
               current_state=FILE_CHECK;
               break;
             }else{
               current_state=READ_IN;
             }
         }else if(current_state== READ_IN){
           i++;
           m_binarySimFile >> m_state;
            if(i % 20==0){
               LOG(m_pThreadLog,  "---> File loader buffering state: " << m_state->m_t <<"..."<<std::endl);
           }
           current_state = FILE_CHECK;
         }
         else if(current_state== FINALIZE_AND_BREAK){
            // Buffer full, break here
            m_pThreadLog->logMessage("---> File pre-buffering finished...");
            break;
         }
      }
      // =========================================================================================


      // wait for caller thread;
      m_barrier_start.wait();

      while(!isLoaderThreadToBeStopped() && current_state != EXIT)
      {

         if(current_state == LOAD_NEXT_FILE ){
            //Load next file
            if(m_currentFileIndex != m_simFileList.size()-1){
                if(!loadNextFile()){
                    current_state = EXIT;
                }else{
                    LOG(m_pThreadLog, "---> SimFile "<<m_currentFileIndex<<"/"<<m_simFileList.size()<<" loaded: Number of States = " << m_binarySimFile.getNStates() << std::endl;);
                    current_state = READ_IN;
                }
            }else{
                 // Write end flag to state if that was the last file!
                LOG(m_pThreadLog,  "---> Write endstate to m_state m_t:" << m_state->m_t <<std::endl;);
                m_state->m_StateType = DynamicsState::ENDSTATE;
                current_state = FINALIZE_AND_BREAK;
            }
         }
         else if(current_state== FILE_CHECK){
            if(m_binarySimFile.isGood()){
               current_state = MOVE_POINTER;
            }else{
                current_state = LOAD_NEXT_FILE;
            }
         }
         else if(current_state== MOVE_POINTER){
            m_state = m_pStatePool->advanceLoadBuffer(bMovedBuffer);
            if(bMovedBuffer){
               current_state = READ_IN;
            }
         }else if(current_state== READ_IN){
            m_binarySimFile >> m_state;
               /*
               LOG(m_pThreadLog,  "Loaded m_t:" << m_state->m_t <<endl;);
                */
           current_state = FILE_CHECK;
         }else if(current_state== FINALIZE_AND_BREAK){
            //Move pointer once more! To make this state avalibale to the vis pointer!
            m_state = m_pStatePool->advanceLoadBuffer(bMovedBuffer);
            if(bMovedBuffer){
               m_pThreadLog->logMessage("---> Buffering reached end of file...");
               // Try to load next file;
               current_state=LOAD_NEXT_FILE;
            }
         }
      }

      // Close file
      unloadFile();


     m_pThreadLog->logMessage("---> PlaybackLoader: LoaderThread leaving...");
     setLoaderThreadRunning(false);
}

template<typename TStatePool>
void PlaybackLoader<TStatePool>::reset()
{
  m_state = m_pStatePool->getLoadBuffer();
}


template<typename TStatePool>
void PlaybackLoader<TStatePool>::setLoaderThreadRunning( bool value )
{
  boost::mutex::scoped_lock l(m_bLoaderThreadRunning_mutex);
  m_bLoaderThreadRunning = value;
}
template<typename TStatePool>
bool PlaybackLoader<TStatePool>::isLoaderThreadRunning()
{
  boost::mutex::scoped_lock l(m_bLoaderThreadRunning_mutex);
  return m_bLoaderThreadRunning;
}

template<typename TStatePool>
void PlaybackLoader<TStatePool>::setThreadToBeStopped( bool stop )
{
  boost::mutex::scoped_lock l(m_bThreadToBeStopped_mutex);
  m_bThreadToBeStopped = stop;
}

template<typename TStatePool>
bool PlaybackLoader<TStatePool>::isLoaderThreadToBeStopped()
{
  boost::mutex::scoped_lock l(m_bThreadToBeStopped_mutex);
  return m_bThreadToBeStopped;
}



template<typename TStatePool>
bool PlaybackLoader<TStatePool>::loadNextFile()
{
  using namespace boost::filesystem;

  m_currentFileIndex++;

  if(m_currentFileIndex != 0){
    m_currentFileIt++;
  }

  if(m_currentFileIt!=m_simFileList.end() ){
    if( m_currentFileIt->empty()){
        m_pThreadLog->logMessage("---> PlaybackLoader:: Path in File list is empty! This should not happen!");
        return false;
    }
  }else{
        m_pThreadLog->logMessage("---> PlaybackLoader:: You have no files selected, please rescan the directory...");
        return false;
  }


  if(boost::filesystem::exists(*m_currentFileIt)){
    if(is_regular_file(*m_currentFileIt)){
      if(!boost::filesystem::is_empty(*m_currentFileIt)){

        // Try to load the file
        m_binarySimFile.close();
        if(m_binarySimFile.openRead(*m_currentFileIt,NDOFqBody,NDOFuBody,m_nSimBodies,m_bReadFullState))
        {
          return true;
        }else{
           LOG(m_pThreadLog, "---> PlaybackLoader:: Could not open file: " << m_currentFileIt->string()
               << std::endl << "---> File errors: " <<std::endl<< m_binarySimFile.getErrorString(););
        }
      }
      else{
        m_pThreadLog->logMessage("---> PlaybackLoader:: File Error:  File is empty...");
      }
    }
    else{
     m_pThreadLog->logMessage("---> PlaybackLoader:: File Error: File is not a regular file...");
    }
  }
  else{
    m_pThreadLog->logMessage("---> PlaybackLoader:: File Error: File does not exist...");
  }

  return false;
}


template<typename TStatePool>
void PlaybackLoader<TStatePool>::unloadFile()
{
  m_binarySimFile.close();
  m_pThreadLog->logMessage("---> PlaybackLoader:: File closed...");
}

#endif
