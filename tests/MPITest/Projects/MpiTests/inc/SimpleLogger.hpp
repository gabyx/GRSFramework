// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef SimpleLogger_hpp
#define SimpleLogger_hpp

#include <iostream>
#include <string>
#include <sstream>

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

#include <boost/thread.hpp>

#include "AssertionDebug.hpp"
#include "Singleton.hpp"

namespace Logging {


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

class LogSinkFile : public LogSink {
private:
    boost::filesystem::ofstream m_fileStream;
public:
    LogSinkFile(const std::string & sink_name, boost::filesystem::path filePath = "" );
    ~LogSinkFile();
};

class LogSinkCout : public LogSink {
public:
    LogSinkCout(const std::string & sink_name);
    ~LogSinkCout();
};


class Log {
protected:

    std::string m_logName;
    boost::mutex m_busy_mutex;

    // Can have multiple streams!
    std::vector<LogSink *> m_sinkList;

    // Push stringstream to all sinks!

    inline void writeOut(std::stringstream & s){
        boost::mutex::scoped_lock l(m_busy_mutex);
        std::vector<LogSink *>::iterator it;
        for(it=m_sinkList.begin(); it != m_sinkList.end(); it++) {
            (*(*it)) << s;
        }
    };

    friend class expression;

public:

    virtual ~Log();
    Log(std::string log_name);

    template<typename T>
    void logMessage(const T & str){
        boost::mutex::scoped_lock l(m_busy_mutex);
        std::vector<LogSink *>::iterator it;
        for(it=m_sinkList.begin(); it != m_sinkList.end(); it++) {
            (*(*it)) << str;
            (*(*it)) << std::endl;
        }
    }

    void logMessage(std::stringstream & str);


    bool addSink(LogSink * sink);
    bool removeSink(std::string sink_name);

    std::string getName();

    class expression;

    template<typename T>
    Log::expression operator<<(const T & t) {
        // makes a chain of temporarys and writes all into a string stream and then writes all out!
        std::stringstream * s=new std::stringstream();
        *s << t; // Push first value into stream;
        return Log::expression(*this, s);
    };

    Log::expression operator<<(std::ostream&(*f)(std::ostream&) ) {
        // makes a chain of temporarys and writes all into a string stream and then writes all out!
        std::stringstream * s=new std::stringstream();
        *s << f; // Push first value into stream;
        return Log::expression(*this, s);
    };

    // Expression to write all " myLogSink << a << b << c " first into a stringstream and the flush!
    class expression {
    public:

        expression(Log & _log, std::stringstream *_s);
        // Destructor pushes message!
        ~expression();

        template <typename T>
        expression operator<<(const T & t) {
            this->flag = true;
            *(this->s) << t; // Push message
            return expression(this->m_log , this->s);
        };

        // For std::endl;
        expression operator<<( std::ostream&(*f)(std::ostream&) );

    private:
        std::stringstream *s;
        bool flag; // When flag is false, the flush gets executed in deconstructor!
        Log & m_log;
    };

};




class LogManager : public Utilities::Singleton<LogManager> {
private:
    typedef std::map<std::string, Log *> LogListType;
    typedef std::map<std::string, Log *>::iterator LogListIteratorType;
    LogListType m_logList;

    boost::mutex m_busy_mutex;

public:

    ~LogManager();

     Log* createLog(const std::string & name, bool toConsole, bool toFile, boost::filesystem::path filePath);

    void destroyLog(const std::string &name);
    void destroyLog(const Log * log);

    void registerLog(Log * log);

    Log * getLog(const std::string & name);
    bool  existsLog(const std::string & name);
};

};

#endif

