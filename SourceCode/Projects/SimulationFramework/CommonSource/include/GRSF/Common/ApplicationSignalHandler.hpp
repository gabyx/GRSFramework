#ifndef GRSF_Common_ApplicationSignalHandler_hpp
#define GRSF_Common_ApplicationSignalHandler_hpp

#include <csignal>
#include <unordered_set>
#include <unordered_map>
#include <deque>
#include <initializer_list>
#include "GRSF/Common/Singleton.hpp"


/**
*  @brief Signal Handling for the Application
*/
class ApplicationSignalHandler: public Utilities::Singleton<ApplicationSignalHandler> {
public:

    ApplicationSignalHandler(std::initializer_list<int> signals){
        // register all signals with the global handler
        for(auto s : signals){ signal(s, ApplicationSignalHandler::globalDispatcher); }
    }

    struct CallBackWrapper{
        std::function<bool(void)> m_func;

        inline bool operator()(){
            return m_func();
        }
        inline bool operator==
        struct Hasher{
            std::size_t operator()(CallBackWrapper & c){
                return getAddress(c.m_func);
            }
        };
    };

    using CallbackType = CallBackWrapper;

    void registerSignalCallback(int signal, CallbackType & callBack ){
        auto address = getAddress(callBack);

        auto & addresses = m_signalHandlersAddresses[signal];
        auto & callbacks = m_signalHandlers[signal];

        auto it = addresses.find(address);
        if(it == addresses.end()){
            addresses.insert(callBack);
            callbacks.push_front(callBack);
        }else{
            ERRORMSG("This signal: " << signal <<" with callback @" << getAddress(callBack) << " is already registered!")
        }
    }

    void unregisterSignalCallback(int signal, CallbackType & callBack ){
        auto it = m_signalHandlers.find(signal);
        if(it != m_signalHandlers.end()){
            auto c = it->second.find(callBack);
            if(c!=it->second.end()){
                it->second.erase(c);
            }else{
                ERRORMSG("Could not unregister signal handler with signal: " << signal << getAddress(callBack))
            }
        }else{
            ERRORMSG("Could not unregister signal handler with signal: " << signal << getAddress(callBack))
        }
    }



private:

    static size_t getAddress(std::function<bool(void)> f) {
        typedef bool (fnType)(void);
        fnType ** fnPointer = f.template target<fnType*>();
        return (size_t) *fnPointer;
    }

    static void globalDispatcher(int signum){

        ApplicationSignalHandler & s = ApplicationSignalHandler::getSingleton();

        bool signalHandled = false;
        // Catched signal
        // Handling all signals and dispatching to the registered signal handlers
        auto it = s.m_signalHandlers.find(signum);
        if(it!=s.m_signalHandlers.end()){
            for( auto & callBack : it->second){  // loop over all callbacks till one handles the signal!
                if(callBack()){
                    signalHandled = true;
                    break;
                };
            }
        }

       std::cerr << "---> Caught signal: " << signum << " which was not handled by the application! Exit!" << std::endl;
       exit(EXIT_FAILURE);
    }

    std::unordered_map<int, std::deque<CallbackType> > m_signalHandlers;

    std::unordered_map<int, std::unordered_set<size_t> > m_signalHandlersAddresses;
};


#endif
