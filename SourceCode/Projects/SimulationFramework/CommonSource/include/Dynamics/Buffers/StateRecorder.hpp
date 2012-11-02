#ifndef STATE_RECORDER_HPP
#define STATE_RECORDER_HPP

#include <string>
#include <Eigen/Dense>

#include "FileManager.hpp"

#include "TypeDefs.hpp"

#include "CommonFunctions.hpp"
#include "DynamicsState.hpp"
#include "LogDefines.hpp"
#include "MultiBodySimFile.hpp"
#include "SimpleLogger.hpp"

/**
* @ingroup StatesAndBuffers
* @brief This is the StateRecorder class which records states to a MultiBodySimFile.
* @{
*/
template <typename TDynamicsSystemType>
class StateRecorder
{
public:
  DEFINE_DYNAMICSSYTEM_CONFIG_TYPES_OF(TDynamicsSystemType::DynamicsSystemConfig)
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  StateRecorder(const unsigned int nSimBodies);

  ~StateRecorder();
  bool createSimFile(boost::filesystem::path file_path);
  bool createSimFileCopyFromReference(boost::filesystem::path new_file_path, boost::filesystem::path ref_file_path);
  void closeSimFile();

  //void addState(const DynamicsState<TLayoutConfig> * state);
  /*void writeAllStates();*/

  void write(const DynamicsState<LayoutConfigType>* value);

  StateRecorder<TDynamicsSystemType> & operator << (const DynamicsState<LayoutConfigType>* value);

protected:

  Logging::Log * m_pSimulationLog;

  //void scanAllSimDataFiles(boost::filesystem::path path_name, bool with_SubDirs = false);
  // All files created have the name "<m_fileNamePrefix>_<m_fileIdCounter>.sim"
  // If files already exist in the folder m_fileIdCounter is set to the actual number which does not exist already!
  //std::vector<boost::filesystem::path> m_SimFilePaths;
  //Ogre::StringVector m_SimFileNames;


  MultiBodySimFile    m_BinarySimFile;

  //std::vector< DynamicsState<LayoutConfigType> >	m_states;

  unsigned int m_nSimBodies;
};

/** @} */



template<typename TDynamicsSystemType>
StateRecorder<TDynamicsSystemType>::StateRecorder(const unsigned int nSimBodies):
m_BinarySimFile(LayoutConfigType::LayoutType::NDOFqObj, LayoutConfigType::LayoutType::NDOFuObj)
{
   m_nSimBodies = nSimBodies;

   //Check if LogManager is available
   Logging::LogManager * manager = Logging::LogManager::getSingletonPtr();
   m_pSimulationLog = manager->getLog("SimulationLog");
    if(!m_pSimulationLog){
        // Log does not exist make a new standart log!
        boost::filesystem::path filePath = FileManager::getSingletonPtr()->getLocalDirectoryPath();
        filePath /= GLOBAL_LOG_FOLDER_DIRECTORY;
        filePath /= "StateRecorderLog.log";
        m_pSimulationLog = manager->createLog("StateRecorderLog",true,true,filePath);
    }

}

template<typename TDynamicsSystemType>
StateRecorder<TDynamicsSystemType>::~StateRecorder()
{
  DECONSTRUCTOR_MESSAGE
  closeSimFile();
}


template<typename TDynamicsSystemType>
bool StateRecorder<TDynamicsSystemType>::createSimFile(boost::filesystem::path file_path)
{
  m_pSimulationLog->logMessage("Record to Sim file at: " + file_path.string());
  if(m_BinarySimFile.openSimFileWrite(file_path,m_nSimBodies)){
    return true;
  }
  return false;
}

template<typename TDynamicsSystemType>
bool StateRecorder<TDynamicsSystemType>::createSimFileCopyFromReference(boost::filesystem::path new_file_path, boost::filesystem::path ref_file_path)
{

   MultiBodySimFile tmpFile(NDOFqObj,NDOFuObj);
   bool fileOK = tmpFile.openSimFileRead(ref_file_path,m_nSimBodies,false); //Open file to see if this file fits our simulation!!
   tmpFile.closeSimFile();

   if(fileOK){
      m_pSimulationLog->logMessage("Copy file:" + ref_file_path.string() + " to: " + new_file_path.string());
      FileManager::getSingletonPtr()->copyFile(ref_file_path,new_file_path,true);

      m_pSimulationLog->logMessage("Record and append to Sim file at: " + new_file_path.string());
      if(m_BinarySimFile.openSimFileWrite(new_file_path,m_nSimBodies,false)){ //APPEND!
         return true;
      }
   }

  return false;
}

template<typename TDynamicsSystemType>
StateRecorder<TDynamicsSystemType> & StateRecorder<TDynamicsSystemType>::operator << (const DynamicsState<LayoutConfigType>* value)
{
  m_BinarySimFile << (value);
}


template<typename TDynamicsSystemType>
void StateRecorder<TDynamicsSystemType>::closeSimFile()
{
  m_BinarySimFile.closeSimFile();
}

//template<typename TLayoutConfig>
//void StateRecorder<TLayoutConfig>::addState(const DynamicsState<TLayoutConfig> * state)
//{
//  m_states.push_back(*state);
//  return;
//}


//template <typename TLayoutConfig>
//void StateRecorder<TLayoutConfig>::writeAllStates()
//{
//
//  if(createSimFile()){
//    for(std::vector<double>::iterator it = m_states.begin(); it != m_states.end(); it++)
//    {
//      m_BinarySimFile << (*it);
//    }
//  }
//}


template <typename TDynamicsSystemType>
void StateRecorder<TDynamicsSystemType>::write( const DynamicsState<LayoutConfigType>* value )
{
  m_BinarySimFile << value;
}

#endif
