// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_common_ApplicationSignalHandler_hpp
#define GRSF_common_ApplicationSignalHandler_hpp

#include <cstring>
#include <csignal>
#include <algorithm>
#include <unordered_map>
#include <deque>
#include <functional>

#include <initializer_list>
#include "GRSF/common/Singleton.hpp"
#include "GRSF/common/Asserts.hpp"

/**
*  @brief Signal Handling for the Application
*  Good overview about signals: https://www.linuxprogrammingblog.com/all-about-linux-signals?page=show
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

    ApplicationSignalHandler(std::initializer_list<int> signals)
        : m_currentStack(&m_signalStacks[1]), m_handlingStack(&m_signalStacks[0]), m_handlingStackIdx(0)
    {
        // register all signals with the global handler
        for(auto signum: signals){
            // make an empty callback stack
            m_signalHandlers.emplace(signum,std::deque<CallbackType>{});
        }

        for(auto signum: signals){
            installGlobalDispatcher(signum);
        }


    }

    void registerCallback(const std::initializer_list<int> & signals,
                          const FuncType & callBack,
                          const std::string & name )
    {
        CallbackType c(callBack,name);
        for(auto signum : signals){

            auto callBacksIt = m_signalHandlers.find(signum);
            if(callBacksIt == m_signalHandlers.end()){
                GRSF_ERRORMSG("You cannot register signal type: " << signum << " because not added to SignalHandler at instantiation!")
            }
            auto & callBacks = callBacksIt->second;
            auto callbackIt  = std::find( callBacks.begin(), callBacks.end(), name); // find already exiting callback
            if(callbackIt == callBacks.end()){
                callBacks.push_front(c);
            }else{
                GRSF_ERRORMSG("This signal: " << signum <<" with callback name" << c.getName() << " is already registered!")
            }

        }
    }

    void registerCallback(int signum, const FuncType & callBack, const std::string & name ){
        registerCallback( std::initializer_list<int>{signum},callBack,name );
    }

    void unregisterCallback(int signum, const std::string & name){

        auto it = m_signalHandlers.find(signum);
        if(it != m_signalHandlers.end()){
            auto & callBacks = it->second;
            auto callbackIt  = std::find( callBacks.begin(), callBacks.end(), name); // find already existing callback
            if(callbackIt != callBacks.end()){
                callBacks.erase(callbackIt);
            }else{
                GRSF_ERRORMSG("No callback: " << name << " to unregister!")
            }
        }else{
            GRSF_ERRORMSG("No callbacks for signal: " << signum << " because not added to SignalHandler at instantiation!")
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


    /** trys to unregisters all callbacks for signal \p signum */
    void unregisterCallbacks(int signum){

        for (auto & sH : m_signalHandlers){
            auto & callBacks = sH.second;
            callBacks.clear();
        }
    }


    void installDefaultSignal(int signum){

        auto it = m_signalHandlers.find(signum);
        if(it == m_signalHandlers.end()){
             GRSF_ERRORMSG("You cannot install default handler for signal: " << signum << "because not registered in CTOR")
        }

        class sigaction act;

        memset (&act, '\0', sizeof(act)); // set to zero
        act.sa_handler = SIG_DFL;

        if (sigaction(signum, &act, NULL) < 0) {
            GRSF_ERRORMSG("Could not install default signal: " << signum);
        }
    }

    void installGlobalDispatcher(int signum){


        auto it = m_signalHandlers.find(signum);
        if(it == m_signalHandlers.end()){
             GRSF_ERRORMSG("You cannot install global dispatcher for signal: " << signum << "because not registered in CTOR")
        }


        // block all signals when executing globalDispatch
        sigset_t block_mask;
        sigemptyset (&block_mask);
        for(auto & s: m_signalHandlers){
            sigaddset(&block_mask, s.first);
        }

        class sigaction act;
        memset (&act, '\0', sizeof(act)); // set to zero
        act.sa_sigaction = &ApplicationSignalHandler::globalSigCatcher;
        act.sa_mask = block_mask;

        if (sigaction(signum, &act, NULL) < 0) {
            GRSF_ERRORMSG("Could not install globalSigCatcher for signal: " << signum);
        }
    }

    void handlePendingSignals(){

        // first swap the signalStacks, getting the currentStack exclusively for handling
        m_handlingStackIdx = (m_handlingStackIdx == 1)? 0 : 1; // atomic write (sig catcher could jump in here in between ... does not matter)
        m_handlingStack = &m_signalStacks[m_handlingStackIdx];


        // Handling all signals in the current handling stack and dispatching to the registered callbacks (LIFO order)
        for( auto signum : *m_handlingStack){
            std::cerr << "ApplicationSignalHandler:: handling signal: " << signum << std::endl;

            auto it = m_signalHandlers.find(signum);
            if(it!=m_signalHandlers.end()){

                if(it->second.size()==0){
                    WARNINGMSG(false, "---> Caught signal: " << signum << " which was not handled by the application, because no call back for this signal)!" );
                }else{
                    for( auto & callBack : it->second){  // loop over all callbacks
                        callBack(signum);
                    }
                }

            }else{
                WARNINGMSG(false,"---> Caught signal: " << signum << " which was not handled by the application because signal not registered!");
            }
        }

        // Clear all entries in the handling stack
        m_handlingStack->clear();
    }


private:


    static void globalSigCatcher(int signum, siginfo_t *siginfo, void *context){

          ApplicationSignalHandler & s = ApplicationSignalHandler::getSingleton();

          // see in which stack to push signal
          s.m_currentStack = (s.m_handlingStackIdx == 1)? &s.m_signalStacks[0] : &s.m_signalStacks[1];
          s.m_currentStack->push_back(signum);
    }

    std::unordered_map<int, std::deque<CallbackType> > m_signalHandlers; ///< The call back stack for each signal

    /** All received signals
     * We have two stacks, one is the current stack \p m_currentStack which gets updated when ever a signal appears we listen to
     * and the second stack \p m_handlingStack is the stack which gets handled and is emptied when handling completes.
     * the atomic \p m_handlingStackIdx makes sure no race condition happens. (e.g. when writing and just a async signal happens)
     */
    std::deque<int> m_signalStacks[2];

    std::deque<int> * m_handlingStack;
    std::deque<int> * m_currentStack;  ///< global dispatch pushes received signals into this stack


    volatile sig_atomic_t m_handlingStackIdx;

};


#endif
