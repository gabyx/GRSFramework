
#include "GRSF/Dynamics/Buffers/StateRecorder.hpp"

StateRecorder::StateRecorder(const unsigned int nSimBodies){
    m_nSimBodies = nSimBodies;

    //Check if LogManager is available
    Logging::LogManager & manager = Logging::LogManager::getSingleton();
    m_pSimulationLog = manager.getLog("SimulationLog");
    if(!m_pSimulationLog) {
        // Log does not exist make a new standart log!
        boost::filesystem::path filePath = FileManager::getSingleton().getLocalDirectoryPath();
        filePath /= GLOBAL_LOG_FOLDER_DIRECTORY;
        filePath /= "StateRecorderLog.log";
        m_pSimulationLog = manager.createLog("StateRecorderLog",true,true,filePath);
    }

}


StateRecorder::~StateRecorder() {
    DECONSTRUCTOR_MESSAGE
    closeAll();
}



bool StateRecorder::createSimFile(boost::filesystem::path file_path) {
    m_pSimulationLog->logMessage("---> Record to Sim file at: " + file_path.string());
    if(m_binarySimFile.openWrite(file_path,
                                 LayoutConfigType::LayoutType::NDOFqBody,
                                 LayoutConfigType::LayoutType::NDOFuBody,
                                 m_nSimBodies)) {
        return true;
    }
    return false;
}


bool StateRecorder::createSimFileCopyFromReference(boost::filesystem::path new_file_path, boost::filesystem::path ref_file_path) {

    MultiBodySimFile tmpFile;
    bool fileOK = tmpFile.openRead(ref_file_path,
                                   false,
                                   LayoutConfigType::LayoutType::NDOFqBody,
                                   LayoutConfigType::LayoutType::NDOFuBody,
                                   m_nSimBodies
                                   ); //Open file to see if this file fits our simulation!!
    tmpFile.close();

    if(fileOK) {
        m_pSimulationLog->logMessage("---> Copy file:" + ref_file_path.string() + " to: " + new_file_path.string());
        FileManager::getSingleton().copyFile(ref_file_path,new_file_path,true);

        m_pSimulationLog->logMessage("---> Record and append to Sim file at: " + new_file_path.string());
        if(m_binarySimFile.openWrite(new_file_path,
                                     LayoutConfigType::LayoutType::NDOFqBody,
                                     LayoutConfigType::LayoutType::NDOFuBody,
                                     m_nSimBodies,
                                     false))
        { //APPEND!
            return true;
        }
    }

    return false;
}


StateRecorder & StateRecorder::operator << (const DynamicsState* value) {
    m_binarySimFile << (value);
    return *this;
}



void StateRecorder::closeAll() {
    m_binarySimFile.close();
}


void StateRecorder::write( const DynamicsState* value ) {
    m_binarySimFile << value;
}


