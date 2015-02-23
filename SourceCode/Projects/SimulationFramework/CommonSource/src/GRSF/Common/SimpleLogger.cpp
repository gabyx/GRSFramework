#include "GRSF/Common/SimpleLogger.hpp"


using namespace Logging;

// LogSink =================================================
std::string LogSink::getName() {
    return m_sinkName;
};

void LogSink::operator<<(std::stringstream & s) {
    (*m_pOutStream) << s.rdbuf();
    s.seekg(0,std::ios_base::beg); // Reset because we might reuse this stringstream!
    if(m_pOutStream->fail()) {
        ASSERTMSG(false,"m_pOutStream has failed()!");
        m_pOutStream->clear();
    }
    m_pOutStream->flush();
}

void LogSink::operator<<(const std::string & s) {
    (*m_pOutStream) << s;
}

//std::endl;
void LogSink::operator<<( std::ostream&(*f)(std::ostream&) ) {
    (*m_pOutStream) << f;
}

LogSink::LogSink(const std::string & name): m_sinkName(name), m_pOutStream(nullptr) {};

LogSink::~LogSink() {
    /* Derived class deletes the stuff */
    //std::cout <<"Delete SINK:"<<std::endl;
};



/**
*   maxSize = 0:
*/
LogSinkFile::LogSinkFile(const std::string & sink_name,
                         boost::filesystem::path filePath,
                         std::streamsize rollSize): LogSink(sink_name) {

    m_rollSize = rollSize;

    if(filePath.empty()){
            filePath = "./Logs";
            filePath /= this->getName() + "fileSink.log";
    }

    if(filePath.has_parent_path() && !boost::filesystem::exists(filePath.parent_path())){
        boost::filesystem::create_directories(filePath.parent_path());
    }

    m_fileStream.open(filePath.string(),std::ofstream::trunc);
    if(!m_fileStream.is_open()) {
        ERRORMSG("LogSinkFile: " << this->getName() << " could not be opened at location: " << filePath.string());
    }
    m_pOutStream = &m_fileStream;
};

std::streampos LogSinkFile::getCurrentWritePosition(){
    return m_fileStream.tellp();
}


void LogSinkFile::doRollToStart(std::streamsize rollSize){
    std::streamsize usedRollSize = m_rollSize;
    // use external rollsize if not zero
    if(rollSize>0){
        usedRollSize = rollSize;
    }

    if( getCurrentWritePosition() >= usedRollSize){
            // do reset write pointer
            m_fileStream << "ROLL TO BEGIN" << std::endl;
            m_fileStream.seekp(0);
    }
}

LogSinkFile::~LogSinkFile() {
    //std::cout <<"Delete LogSinkFile:"<<std::endl;
    m_fileStream.flush();
    m_fileStream.close();
    m_pOutStream = nullptr;
};



// LogSinkCout =================================================
LogSinkCout::LogSinkCout(const std::string & sink_name): LogSink(sink_name) {
    m_pOutStream = &std::cout;
};
LogSinkCout::~LogSinkCout() {
    //std::cout <<"Delete LogSinkCout:"<<std::endl;
};


// Log =================================================
Log::~Log() {
    for(unsigned int i=0; i<m_sinkList.size(); i++) {
        delete m_sinkList[i];
    }
};

Log::Log(std::string log_name) {
    m_logName = log_name;
}

void Log::logMessage(std::stringstream & str) {
    writeOut(str);
}


bool Log::addSink(LogSink * sink) {
    std::lock_guard<std::mutex> l(m_busy_mutex);
    for(unsigned int i=0; i<m_sinkList.size(); i++) {
        if(m_sinkList[i]==sink) {
            return false;
        }
    }
    m_sinkList.push_back(sink);
    return true;
}

bool Log::removeSink(std::string sink_name) {
    std::lock_guard<std::mutex> l(m_busy_mutex);
    std::vector<LogSink *>::iterator it;
    for(it=m_sinkList.begin(); it!=m_sinkList.end(); ++it) {
        if((*it)->getName()==sink_name) {
            it=m_sinkList.erase(it);
            return true;
        }
    }
    return false;
}

std::string Log::getName() {
    return m_logName;
}

// Log::expression =================================================
Log::expression::expression(Log & _log, std::stringstream &_s, bool lastWasEndl)
: m_s(_s), /*m_flag(true), */m_lastWasEndl(lastWasEndl), m_log(_log) {};

// Destructor pushes message!
Log::expression::~expression() {
        m_log.writeOut(m_s); //Push string stream to log
        // if last pushed stuff was an endl then the log starts with a new line!
        m_log.m_newLine = m_lastWasEndl;
}

// For std::endl;
Log::expression &  Log::expression::operator<<( std::ostream&(*f)(std::ostream&) ) {
    m_s << f; // Apply the manipulator to the stream

    if(f == static_cast<std::ostream&(&)(std::ostream&)>(std::endl)){
           m_lastWasEndl = true;
    }

    return *this;
}


// LogManager =======================================================

LogManager::LogManager() {
    m_globalClock.start();
}

LogManager::~LogManager() {
    LogListIteratorType it;
    for(it=m_logList.begin(); it!= m_logList.end(); ++it) {
        if((*it).second) {
            delete (*it).second; //Delete all logs!
        }
    }
};

Log* LogManager::createLog(const std::string & name, bool toConsole, bool toFile, boost::filesystem::path filePath, bool useTimer) {
    Log * pLog = new Log(name);

    if(useTimer){
        pLog->setTimer(&m_globalClock);
    }

    if(toConsole) {
        pLog->addSink(new LogSinkCout(name + std::string("-CoutSink")));

    }
    if(toFile) {
        pLog->addSink(new LogSinkFile(name + std::string("-FileSink"),filePath));
    }
    registerLog(pLog);
    return pLog;
};

void LogManager::destroyLog(const std::string &name) {
    std::lock_guard<std::mutex> l(m_busy_mutex);

    LogListIteratorType it = m_logList.find(name);
    ASSERTMSG(it != m_logList.end(),"This Log does not exist!");
    if( it != m_logList.end() ) {
        ASSERTMSG(it->second != nullptr,"This Log has Null Pointer!");
        if(it->second) {
            delete it->second; // Delete Log
        }
        // Remove from list;
        m_logList.erase(it);
    }
};

void LogManager::destroyLog(Log * & log) {
    std::lock_guard<std::mutex> l(m_busy_mutex);
    LogListIteratorType it;
    for(it = m_logList.begin(); it != m_logList.end(); it++){
            if(it->second && it->second == log) {
                delete it->second; // Delete Log
                log = nullptr;
            }
            // Remove from list;
            m_logList.erase(it);
            return;
    }
    ASSERTMSG(it != m_logList.end(),"This Log does not exist!");
};

void LogManager::registerLog(Log * log, bool useTimer) {
    ASSERTMSG(log != nullptr,"This Log has Null Pointer!");
    std::lock_guard<std::mutex> l(m_busy_mutex);

    if(useTimer){
        log->setTimer(&m_globalClock);
    }

    std::pair<LogListIteratorType, bool> res =  m_logList.insert(std::pair<std::string, Log* >(log->getName(), log));
    ASSERTMSG(res.second,"LogSink has already been added! :" << log->getName());
};

Log * LogManager::getLog(const std::string & name) {
    std::lock_guard<std::mutex> l(m_busy_mutex);
    LogListIteratorType it = m_logList.find(name);
    ASSERTMSG(it != m_logList.end(),"This Log does not exist!: "<<name);
    ASSERTMSG(it->second != nullptr,"This Log has Null Pointer!: "<<name);
    if(it == m_logList.end()) {
        return nullptr;
    }
    return it->second;
};

bool LogManager::existsLog(const std::string & name) {
    std::lock_guard<std::mutex> l(m_busy_mutex);
    LogListIteratorType it = m_logList.find(name);
    if(it == m_logList.end()){
        return false;
    }
    return true;
};

void LogManager::rollAllLogs(std::streamsize rollSize) {
    std::lock_guard<std::mutex> l(m_busy_mutex);

    // if rollSize = 0, each file is treated seperately
    // if rollSize > 0, this size is used for all files

    for(auto & l : m_logList){

        auto & sinks = l.second->getSinks();
        for(auto &s : sinks){
            Logging::LogSinkFile* f = dynamic_cast<Logging::LogSinkFile*>(s);
            if(f){
                f->doRollToStart(rollSize);
            }
        }

    }
};
