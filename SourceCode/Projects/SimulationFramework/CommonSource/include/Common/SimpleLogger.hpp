#ifndef SimpleLogger_hpp
#define SimpleLogger_hpp

#include <fstream>
#include <vector>
#include <unordered_map>
#include <string>
#include <sstream>
#include <mutex>

#include <boost/filesystem.hpp>

#include "CPUTimer.hpp"
#include "CommonFunctions.hpp"

#include "AssertionDebug.hpp"
#include "Singleton.hpp"

namespace Logging {

#define LOGGING_TIMEFORMAT "%8.3f::"
#define LOGGING_TIMESPACES "        ::"

class LogSink {
protected:
    std::string m_sinkName;
    std::ostream* m_pOutStream;

public:

    std::string getName();

    void operator<<(std::stringstream & s);
    void operator<<(const std::string & s);
    //std::endl;
    void operator<<( std::ostream&(*f)(std::ostream&) );


    LogSink(const std::string & name);

    virtual ~LogSink();
};

/** File Sink which does not roll itself, LogManager needs to do this!, defaultRollSize default to 5 MiB */
class LogSinkFile : public LogSink {
private:
    static const std::streamsize defaultRollSize = 1<<20;

    std::ofstream m_fileStream;
    std::streamsize m_rollSize = 0; ///< maximum size of file when the file should be rolled
public:
    LogSinkFile(const std::string & sink_name, boost::filesystem::path filePath = "",
                std::streamsize  rollSize =  defaultRollSize);

    /** Rolls to the start of the file if limit rollSize or internal m_rollSize is reached */
    void doRollToStart(std::streamsize rollSize = 0);
    std::streampos getCurrentWritePosition();
    ~LogSinkFile();
};

class LogSinkCout : public LogSink {
public:
    LogSinkCout(const std::string & sink_name);
    ~LogSinkCout();
};

/**
* Log class which owns multiple sinks and deletes them in dtor!
*/
class Log {
protected:

    CPUTimer * m_time = nullptr; ///< A timer which can be set from outside!

    std::string m_logName;
    std::mutex m_busy_mutex;

    // Can have multiple streams!
    std::vector<LogSink *> m_sinkList;

    // Push stringstream to all sinks!

    void writeOut(std::stringstream & s){
        std::lock_guard<std::mutex> l(m_busy_mutex);
        std::vector<LogSink *>::iterator it;
        for(it=m_sinkList.begin(); it != m_sinkList.end(); ++it) {
            (*(*it)) << s;
        }
    };

    std::stringstream m_s; ///< Temporary stringstream;

    friend class expression;

    bool m_newLine = true; ///< Markes the state where we are at a new beginning of a line -> push time

public:

    virtual ~Log();
    Log(std::string log_name);

    template<typename T>
    void logMessage(const T & str){
        std::lock_guard<std::mutex> l(m_busy_mutex);
        std::vector<LogSink *>::iterator it;
        for(it=m_sinkList.begin(); it != m_sinkList.end(); ++it) {
            if(m_time){ (*(*it)) << Utilities::stringFormat(LOGGING_TIMEFORMAT,m_time->elapsedMin());}
            (*(*it)) << str;
            (*(*it)) << std::endl;
        }
    }

    void logMessage(std::stringstream & str);

    inline void setTimer(CPUTimer * time=nullptr){ m_time = time;}

    const std::vector<LogSink *> & getSinks(){ return m_sinkList;}
    bool addSink(LogSink * sink);
    bool removeSink(std::string sink_name);

    std::string getName();

    class expression;

    template<typename T>
    Log::expression operator<<(const T & t) {
        // makes a chain of temporarys and writes all into a string stream and then writes all out!
        m_s.str("");

        // push time if timer set
        if(m_time && m_newLine){ m_s << Utilities::stringFormat(LOGGING_TIMEFORMAT,m_time->elapsedMin());}
        m_s << t; // Push first value into stream;

        return Log::expression(*this, m_s);
    };

    Log::expression operator<<(std::ostream&(*f)(std::ostream&) ) {
        // makes a chain of temporarys and writes all into a string stream and then writes all out!
        m_s.str("");
        m_s << f; // Push first value into stream;

        if(f == static_cast<std::ostream&(&)(std::ostream&)>(std::endl)){
            return Log::expression(*this, m_s, true);
        }
        return Log::expression(*this, m_s);
    };

    // Expression to write all " myLogSink << a << b << c " first into a stringstream and the flush!
    class expression {
    public:

        expression(Log & _log, std::stringstream &_s, bool lastWasEndl = false);
        // Destructor pushes message!
        ~expression();

        template <typename T>
        expression &  operator<<(const T & t) {
//            m_flag = false;
            if(m_lastWasEndl && m_log.m_time){ m_s << LOGGING_TIMESPACES;}
            m_s << t; // Push message
            m_lastWasEndl = false;
            return *this;
        };

        // For std::endl;
        expression & operator<<( std::ostream&(*f)(std::ostream&) );

    private:
        std::stringstream &m_s;
        bool m_lastWasEndl;
        Log & m_log;
    };

};



/**
* LogManager class which owns all registered logs and deletes them in dtor !
*/
class LogManager : public Utilities::Singleton<LogManager> {
private:
    typedef std::unordered_map<std::string, Log *> LogListType;
    typedef std::unordered_map<std::string, Log *>::iterator LogListIteratorType;
    LogListType m_logList;

    std::mutex m_busy_mutex;
    CPUTimer m_globalClock;

public:



    LogManager();
    ~LogManager();

    Log* createLog(const std::string & name,
                   bool toConsole,
                   bool toFile,
                   boost::filesystem::path filePath,
                   bool useTimer = true);

    void destroyLog(const std::string &name);
    void destroyLog(const Log * log);

    void registerLog(Log * log, bool useTimer = true);

    Log * getLog(const std::string & name);
    bool  existsLog(const std::string & name);

    /** Checks all FileSinks and set the write pointer to the beginning if rollSize is exceeded */
    void rollAllLogs(std::streamsize rollSize = 0);

};

};


#endif

