#ifndef GRSF_Common_ApplicationSignalHandler_hpp
#define GRSF_Common_ApplicationSignalHandler_hpp

#include <csignal>
#include <unordered_map>
#include <deque>
#include <functional>

#include <initializer_list>
#include "GRSF/Common/Singleton.hpp"
#include "GRSF/Common/AssertionDebug.hpp"

/**
*  @brief Signal Handling for the Application
*/
class ApplicationSignalHandler: public Utilities::Singleton<ApplicationSignalHandler> {
public:

    using FuncType = std::function<void(int)>;


    class CallBackWrapper{
    public:

        CallBackWrapper(const FuncType & f , const std::string & name)
            : m_func(f), m_name(name)
        {}

        inline void operator()(int signal) const{
            m_func(signal);
        }

        inline bool operator==(const CallBackWrapper & callback) const{
            return m_name == callback.m_name;
        }

        inline bool operator==(const std::string & name) const{
            return m_name == name;
        }

        std::string getName() const {return m_name;}

    private:
        FuncType m_func;
        std::string m_name;
    };

    using CallbackType = CallBackWrapper;

    ApplicationSignalHandler(std::initializer_list<int> signals){
        // register all signals with the global handler
        for(auto s : signals){
            signal(s, ApplicationSignalHandler::globalDispatcher);
            m_signalHandlers.emplace(s,std::deque<CallbackType>{});
        }
    }

    void registerCallback(const std::initializer_list<int> & signals,
                          const FuncType & callBack,
                          const std::string & name )
    {
        CallbackType c(callBack,name);
        for(auto signal : signals){

            auto callBacksIt = m_signalHandlers.find(signal);
            if(callBacksIt == m_signalHandlers.end()){
                ERRORMSG("You cannot register signal type: " << signal << " because not added to SignalHandler at instantiation!")
            }
            auto & callBacks = callBacksIt->second;
            auto callbackIt  = std::find( callBacks.begin(), callBacks.end(), name); // find already exiting callback
            if(callbackIt == callBacks.end()){
                callBacks.push_front(c);
            }else{
                ERRORMSG("This signal: " << signal <<" with callback name" << c.getName() << " is already registered!")
            }

        }
    }

    void registerCallback(int signal, const FuncType & callBack, const std::string & name ){
        registerCallback( std::initializer_list<int>{signal},callBack,name );
    }

    void unregisterCallback(int signal, const std::string & name){

        auto it = m_signalHandlers.find(signal);
        if(it != m_signalHandlers.end()){
            auto & callBacks = it->second;
            auto callbackIt  = std::find( callBacks.begin(), callBacks.end(), name); // find already existing callback
            if(callbackIt != callBacks.end()){
                callBacks.erase(callbackIt);
            }else{
                ERRORMSG("No callback: " << name << " to unregister!")
            }
        }else{
            ERRORMSG("No callbacks for signal: " << signal << " because not added to SignalHandler at instantiation!")
        }
    }

    /** trys to unregisters from all signals */
    void unregisterCallback( const std::string & name){

        for (auto & sH : m_signalHandlers){
            auto & callBacks = sH.second;
            auto callbackIt  = std::find( callBacks.begin(), callBacks.end(), name ); // find already existing callback
            if(callbackIt != callBacks.end()){
                callBacks.erase(callbackIt);
            }
        }
    }

private:


    static void globalDispatcher(int signum){

        ApplicationSignalHandler & s = ApplicationSignalHandler::getSingleton();
        // Catched signal
        // Handling all signals and dispatching to the registered signal handlers
        auto it = s.m_signalHandlers.find(signum);
        if(it!=s.m_signalHandlers.end()){

            if(it->second.size()==0){
                WARNINGMSG(false, "---> Caught signal: " << signum << " which was not handled by the application, because no call back for this signal)!" );
            }else{
                for( auto & callBack : it->second){  // loop over all callbacks
                    callBack(it->first);
                }
            }
        }else{
            WARNINGMSG(false,"---> Caught signal: " << signum << " which was not handled by the application because signal not registered!");
        }
    }

    std::unordered_map<int, std::deque<CallbackType> > m_signalHandlers;
};


#endif
