#include "SimpleLogger.hpp"


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



// LogSinkFile =================================================
LogSinkFile::LogSinkFile(const std::string & sink_name, boost::filesystem::path filePath): LogSink(sink_name) {



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
    for(int i=0; i<m_sinkList.size(); i++) {
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
    for(int i=0; i<m_sinkList.size(); i++) {
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
Log::expression::expression(Log & _log, std::stringstream *_s) : flag(false), m_log(_log), s(_s) {};

// Destructor pushes message!
Log::expression::~expression() {
    if(flag==false) {
        m_log.writeOut(*s); //Push string stream to log
        delete s;
    }
}

// For std::endl;
Log::expression Log::expression::operator<<( std::ostream&(*f)(std::ostream&) ) {
    this->flag = true;
    *(this->s) << f; // Apply the manipulator to the stream
    return expression(this->m_log , this->s);
}


// LogManager =======================================================
LogManager::~LogManager() {
    LogListIteratorType it;
    for(it=m_logList.begin(); it!= m_logList.end(); ++it) {
        if((*it).second) {
            delete (*it).second; //Delete all logs!
        }
    }
};

Log* LogManager::createLog(const std::string & name, bool toConsole, bool toFile, boost::filesystem::path filePath) {
    Log * pLog = new Log(name);
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

void LogManager::destroyLog(const Log * log) {
    std::lock_guard<std::mutex> l(m_busy_mutex);
    LogListIteratorType it;
    for(it = m_logList.begin(); it != m_logList.end(); it++){
            if(it->second && it->second == log) {
                delete it->second; // Delete Log
            }
            // Remove from list;
            m_logList.erase(it);
            return;
    }
    ASSERTMSG(it != m_logList.end(),"This Log does not exist!");
};

void LogManager::registerLog(Log * log) {
    ASSERTMSG(log != nullptr,"This Log has Null Pointer!");
    std::lock_guard<std::mutex> l(m_busy_mutex);
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

