// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_converters_SimFileConverter_hpp
#define GRSF_converters_SimFileConverter_hpp

#include <iostream>
#include <iomanip>
#include <set>
#include <map>
#include <string>
#include <vector>

#include <pugixml.hpp>

#include <boost/filesystem.hpp>

#include "GRSF/common/LogDefines.hpp"
#include "GRSF/common/TypeDefs.hpp"

#include "GRSF/common/SimpleLogger.hpp"
#include "GRSF/dynamics/general/MultiBodySimFile.hpp"

#include "GRSF/common/ApplicationSignalHandler.hpp"

#include "GRSF/common/ContainerTag.hpp"
#include "GRSF/common/SfinaeMacros.hpp"
#include "GRSF/common/HasMemberFunction.hpp"
#include "GRSF/common/ExpandParameterPack.hpp"

#include "GRSF/common/CPUTimer.hpp"
#include "GRSF/common/CommonFunctions.hpp"
#include "GRSF/common/ProgressBarCL.hpp"

//#include "GRSF/logic/DummyNode.hpp"


class SimFileConverter {
public:

    using XMLNodeType = pugi::xml_node;
    using XMLNodeItType = pugi::xml_node_iterator;
    using XMLAttributeType = pugi::xml_attribute;


protected:

    using StateIdxType = std::size_t;
    struct StateIndex{
        StateIdxType m_idx;
        StateIdxType m_mappedIdx;
        boost::filesystem::path m_outputFile;
    };

    using StateIndicesType = std::vector< StateIndex >;
    using StateListType = std::vector<RigidBodyStateAdd>;

    MultiBodySimFile m_simFile;

    Logging::Log * m_log;

    std::vector<boost::filesystem::path> m_inputFiles;

    std::size_t m_stateCounter; ///< Counting all frames converted
    std::size_t m_bodyCounter;  ///< Counting all bodies converted

    bool m_terminatedByStepper;

    bool m_abort;

public:

    SimFileConverter(const std::vector<boost::filesystem::path> & inputFiles){

        m_inputFiles = inputFiles;

        auto log = "LogicConverter.log";
        m_log = Logging::LogManager::getSingleton().createLog("LogicConverter",true,true,log);

    }

    /** Settings to configure converter loop */
    template<bool _FullState = false>
    struct ConvertSettings{
        static const bool FullState = _FullState; ///< Calls function addState instead of addBodyState multiple times

        /** Providing addState and FullState=true , the implementation in this class which checks if addState is provided and so on
        * is only meaningful when an read iterator is implemented which iterates over each rigid body state
        * TODO implement iterator interface for sim file.
        * Other this implementation is an overkill and this class should only provide addState which directly reads the whole state always
        * if the stepper functors provided to convert need to iterate over the rigid body states they can for sure do so.
        * we leave this overkill implementation as is because iterator interface for simfile is a very good idea, but has no priority now.
        * We can also not hand some sim file read iterators over to addState because, we dont want to read several times for multiple steppers
        * handed to convert. so the only possible way is to read once (by iterators or by reading a full state junk)
        * in this class and call either addState or addBodyState depending
        * on what the user wants
        */
    };


    using DefaultSettings = ConvertSettings<>;

    template<typename TSettings = DefaultSettings, typename... TSimFileStepper >
    void convert( TSimFileStepper &&... simFileStepper)
    {

        StateIdxType startIdx, endIdx, mappedStartIdx, increment;
        boost::filesystem::path perFileOutputFile;

        bool stop;
        LOG(m_log, "---> LogicConverter started:" <<std::endl;);

        // global framecounter
        m_stateCounter = 0;

        // First open the sim file (if .sim extension)
        // if .xml extension (then this is the process file where each simfile and frame index is stored)

        unsigned int fileIdx = 0;
        for(auto file : m_inputFiles) {

            // Range indices
            startIdx = 0;
            endIdx = std::numeric_limits<StateIdxType>::max();
            increment = 1;
            mappedStartIdx = startIdx;

            if(file.extension() == ".xml"){
                // open the xml
                pugi::xml_document xmlDoc;
                xmlDoc.load_file(file.string().c_str());
                auto node = xmlDoc.child("Converter");
                if(!node){
                    ERRORMSG("XML file: " << file << " contains no 'Converter' node!" )
                }

                StateIndicesType stateIndices;

                for(auto n : node.children("File")){

                    std::string uuid = n.attribute("uuid").value();
                    boost::filesystem::path path = n.attribute("simFile").value();


                    if(path.empty()){
                        LOG(m_log,"---> No simFile path given, skip this file!" << std::endl;)
                        continue;
                    }


                    bool useRange = false;
                    auto att = n.attribute("useRange");
                    if(att){
                        if( !Utilities::stringToType(useRange, n.attribute("useRange").value() )  ) {
                            ERRORMSG("---> String conversion to obtain 'fullFile' failed!");
                        }

                        if(useRange){
                            auto att = n.attribute("startIdx");
                            if(att){
                                if( !Utilities::stringToType(startIdx, n.attribute("startIdx").value() )  ) {
                                    ERRORMSG("---> String conversion to obtain 'fullFile' failed!");
                                }
                            }

                            att = n.attribute("endIdx");
                            if(att){
                                std::streamoff endIdxS = -1;
                                if( !Utilities::stringToType(endIdxS, n.attribute("endIdx").value() )  ) {
                                    ERRORMSG("---> String conversion to obtain 'fullFile' failed!");
                                }
                                if(endIdxS>=0){ // if not negative set to parsed value
                                    endIdx = endIdxS;
                                }
                            }


                            att = n.attribute("increment");
                            if(att){
                                if( !Utilities::stringToType(increment, n.attribute("increment").value() )  ) {
                                    ERRORMSG("---> String conversion to obtain 'fullFile' failed!");
                                }
                            }
                            mappedStartIdx = startIdx;
                            att = n.attribute("mappedStartIdx");
                            if(att){
                                if( !Utilities::stringToType(mappedStartIdx, n.attribute("mappedStartIdx").value() )  ) {
                                    ERRORMSG("---> String conversion to obtain 'fullFile' failed!");
                                }
                            }
                        }
                    }

                    // parse default outputFile,
                    perFileOutputFile = "";
                    att = n.attribute("outputFile");
                    if( att ) {
                       perFileOutputFile = att.value();
                    }

                    // parse frame index list (assumed to be sorted! otherwise exception in convertFile)

                    // if not fullFile parse in all states
                    if(!useRange){
                        stateIndices.clear();
                        StateIdxType idx;
                        unsigned int frameIdx;
                        boost::filesystem::path outputFile;
                        for(auto s : n.children("State")){

                            if( !Utilities::stringToType(idx, s.attribute("stateIdx").value() )  ) {
                                    ERRORMSG("---> String conversion to obtain 'stateIdx' failed!");
                            }

                            if( !Utilities::stringToType(frameIdx, s.attribute("mappedIdx").value() )  ) {
                                    ERRORMSG("---> String conversion to obtain 'mappedIdx' failed!");
                            }

                            outputFile = "";
                            auto att = s.attribute("outputFile");
                            if( att ) {
                               outputFile = att.value();
                            }

                            // add to list
                            stateIndices.push_back( StateIndex{idx,frameIdx,outputFile});
                        }


                        // Sort state indices (in case of non sorted input!)
                        std::sort(stateIndices.begin(),stateIndices.end(), [](StateIndex & a, StateIndex & b){ return a.m_idx < b.m_idx ;} );
                        // Check for duplicate indices
                        if(!stateIndices.empty()){
                            for(auto it = stateIndices.begin(); it != (--stateIndices.end()); ++it){
                                if(it->m_idx == std::next(it)->m_idx){
                                    ERRORMSG(" The file: " << file.filename() << " contains duplicate states idx: " << it->m_idx );
                                }
                            }
                        }

                        LOG(m_log,"---> Parsed " << stateIndices.size() << " state of file: " << path << "from XML: " << file.filename() << std::endl;)
                        if( stateIndices.size() > 0 ){
                            convertFile<TSettings,TSimFileStepper...>(path,uuid,
                                                                      stateIndices,startIdx,endIdx,increment,mappedStartIdx,
                                                                      perFileOutputFile,
                                                                      simFileStepper...);
                        }else{
                            LOG(m_log,"---> No states to process..." << std::endl;)
                        }

                    }else{ // use range

                        if(endIdx == std::numeric_limits<StateIdxType>::max()){
                            LOG(m_log,"---> Use state range: [ "<< startIdx << ":"<<increment<<": -1 ) of file: "
                                << path << "from XML: " << file.filename() << std::endl;)
                        }else{
                            LOG(m_log,"---> Use state range: [ "<< startIdx << ":"<<increment<<":" << endIdx << " ) of file: "
                                << path << "from XML: " << file.filename() << std::endl;)
                        }
                        if(startIdx < endIdx ){
                            convertFile<TSettings,TSimFileStepper...>(path,uuid,
                                                                      {},startIdx,endIdx,increment,mappedStartIdx,
                                                                      perFileOutputFile,
                                                                      simFileStepper...);
                        }else{
                            LOG(m_log,"---> No states to process..." << std::endl;)
                        }
                    }

                }

            }else{
                // try to convert as a sim file
                LOG(m_log,"---> Take all states of file: " << file <<  "(read as .sim file)" << std::endl;)
                convertFile<TSettings, TSimFileStepper...>(file, std::to_string(fileIdx) ,
                                                            {},startIdx,endIdx,increment,mappedStartIdx,
                                                            perFileOutputFile, simFileStepper...);
            }

            ++fileIdx;

            stop = false;
            EXPAND_PARAMETERPACK( stop |= details::StepperDispatch<TSimFileStepper>::isStopFileLoop(simFileStepper) )
            if(stop){
                LOG(m_log,"---> Stop file loop set, fileCount: " << fileIdx << " -> exit" << std::endl;)
                break;
            }
        }
    }

protected:

    void callbackAbort(){ m_abort = true; LOG(m_log, "---> Quitting ...:" <<std::endl);}

    struct details{

        /** Define some stuff to check for member functions */
        template<typename T>
        struct hasMemberFunctionB{
            DEFINE_HAS_MEMBER_FUNCTION(addState)
            DEFINE_HAS_MEMBER_FUNCTION(addBodyState)
            static const bool hasFullStateSupport = hasMemberFunction_addState<T,      StateListType & >::value;
            static const bool hasBodyStateSupport = hasMemberFunction_addBodyState<T , RigidBodyStateAdd *>::value;
        };
        template<
            typename T,
            typename Dummy = decltype(ContainerTags::is_container( std::declval<T>()))
        >
        struct hasMemberFunction;

        /** Member function check for container of stepper*/
        template<typename TSimFileStepper>
        struct hasMemberFunction<TSimFileStepper,std::true_type>
            : hasMemberFunctionB< typename std::remove_reference<TSimFileStepper>::type::value_type > {};

        /** Member function check for single stepper */
        template<typename TSimFileStepper>
        struct hasMemberFunction<TSimFileStepper,std::false_type>
            : hasMemberFunctionB< typename std::remove_reference<TSimFileStepper>::type > {};

        template<typename TSimFileStepper,
                 bool FullState,
                 bool hasFullStateSupport = hasMemberFunction<TSimFileStepper>::hasFullStateSupport,
                 bool hasBodyStateSupport = hasMemberFunction<TSimFileStepper>::hasBodyStateSupport>
        struct FullStateOrBodyState;
        /** If stepper class fails to compile here, you either have missing addState or addBodyState function in your stepper
        *   and wrong set options FullState (which needs addState)
        */
        /** Dispatch for FullState */
        template<typename TSimFileStepper,bool hasBodyStateSupport>
        struct FullStateOrBodyState<TSimFileStepper,true,true,hasBodyStateSupport>{
            template<typename StateCont>
            static void apply(SimFileConverter * p, const TSimFileStepper& simFileStepper, StateCont & states){
                details::StepperDispatch<TSimFileStepper>::addState(simFileStepper, states);
                p->m_bodyCounter += states.size();
            }
        };

        /** Dispatch for BodyState */
        template<typename TSimFileStepper, bool hasFullStateSupport>
        struct FullStateOrBodyState<TSimFileStepper,false,hasFullStateSupport,true>{

            template<typename StateCont>
            static void apply(SimFileConverter * p, const TSimFileStepper& simFileStepper, StateCont & states ){
                std::size_t bodyCounter = 0;
                for(auto & bs: states){

                    details::StepperDispatch<TSimFileStepper>::addBodyState(simFileStepper, &bs);

                    ++p->m_bodyCounter;
                    ++bodyCounter; // local counter

                    if(details::StepperDispatch<TSimFileStepper>::isStopBodyLoop(simFileStepper)){
                        LOG(p->m_log,"---> Stop body loop set, bodyCount: " << bodyCounter << " -> exit" << std::endl;)
                        break;
                    }
                }
            }

        };



        template<typename T, typename Dummy = decltype(ContainerTags::is_container( std::declval<T>() /*make reference, no need for CTOR*/ ))>
        struct StepperDispatch;

        #define SIMFILESTEPPER_DISPATCH_FUNC( __name__ ) SIMFILESTEPPER_DISPATCH_FUNC2( __name__ , void )

        #define SIMFILESTEPPER_DISPATCH_FUNC2(__name__, _return_) \
            template<typename... Args> \
            static _return_ __name__(T & t, Args&&... a ){ \
                    t.__name__(std::forward<Args>(a)...); \
            }
        #define SIMFILESTEPPER_DISPATCH_FUNC_STOP( __name__ )   \
            template<typename... Args> \
            static bool __name__(T & t, Args&&... a ){ \
                return t.__name__(std::forward<Args>(a)...); \
            }

        /** Function hooks dispatch for a single stepper functor */
        template<typename T>
        struct StepperDispatch<T,std::false_type>{
            SIMFILESTEPPER_DISPATCH_FUNC(initSimInfo)
            SIMFILESTEPPER_DISPATCH_FUNC(initState)
            SIMFILESTEPPER_DISPATCH_FUNC(addBodyState)
            SIMFILESTEPPER_DISPATCH_FUNC(addState)
            SIMFILESTEPPER_DISPATCH_FUNC(finalizeState)

            /** Stop whole loop if any stepper in the list needs to stop.
             * Making different steppers and stop and not stop in different loops, needs dynamic managing during the loop
             * which is too cumbersome!
             */
            SIMFILESTEPPER_DISPATCH_FUNC_STOP(isStopFileLoop)
            SIMFILESTEPPER_DISPATCH_FUNC_STOP(isStopFrameLoop)
            SIMFILESTEPPER_DISPATCH_FUNC_STOP(isStopBodyLoop)
        };
        #undef SIMFILESTEPPER_DISPATCH_FUNC
        #undef SIMFILESTEPPER_DISPATCH_FUNC2
        #undef SIMFILESTEPPER_DISPATCH_FUNC_STOP

        #define SIMFILESTEPPER_DISPATCH_FUNC_CONT( __name__ ) SIMFILESTEPPER_DISPATCH_FUNC_CONT2( __name__ , void )

        #define SIMFILESTEPPER_DISPATCH_FUNC_CONT2(__name__, _return_) \
            template<typename... Args> \
            static _return_ __name__(T & c, Args&&... a ){ \
                for(auto & t : c){ \
                    t.__name__(std::forward<Args>(a)...); \
                } \
            }

        #define SIMFILESTEPPER_DISPATCH_FUNC_CONT_STOP( __name__ )   \
            template<typename... Args> \
            static bool __name__(T & c, Args&&... a ){ \
                bool stop=false; \
                for(auto & t : c){ \
                    stop |= t.__name__(std::forward<Args>(a)...); \
                }\
                return stop; \
            }
        /** Function hooks dispatch for a list of stepper functors of the same type*/
        template<typename T>
        struct StepperDispatch<T,std::true_type>{
            SIMFILESTEPPER_DISPATCH_FUNC_CONT(initSimInfo)
            SIMFILESTEPPER_DISPATCH_FUNC_CONT(initState)
            SIMFILESTEPPER_DISPATCH_FUNC_CONT(addBodyState)
            SIMFILESTEPPER_DISPATCH_FUNC_CONT(addState)
            SIMFILESTEPPER_DISPATCH_FUNC_CONT(finalizeState)

            SIMFILESTEPPER_DISPATCH_FUNC_CONT_STOP(isStopFileLoop)
            SIMFILESTEPPER_DISPATCH_FUNC_CONT_STOP(isStopFrameLoop)
            SIMFILESTEPPER_DISPATCH_FUNC_CONT_STOP(isStopBodyLoop)
        };
        #undef SIMFILESTEPPER_DISPATCH_FUNC_CONT
        #undef SIMFILESTEPPER_DISPATCH_FUNC_CONT2
        #undef SIMFILESTEPPER_DISPATCH_FUNC_CONT_STOP
    };


    template<typename TSimFileStepper, bool FullState, bool hasFullStateSupport, bool hasBodyStateSupport>
    friend struct details::FullStateOrBodyState;


    /** \p uuid string is a hash for the file path to identify each frame where it came from!*/
    template<typename TSettings, typename... TSimFileStepper>
    void convertFile(const boost::filesystem::path & f,
                     const std::string uuidString ,
                     const StateIndicesType & stateIndices, /* can be empty*/
                     const StateIdxType startIdx, const StateIdxType endIdx, const StateIdxType increment,
                     const StateIdxType mappedStartIdx,
                     const boost::filesystem::path perFileOutputFile,
                     TSimFileStepper&&... simFileStepper)
    {
        bool stop = false;
        LOG(m_log, "---> Converting file:" << f << std::endl;);


        ApplicationSignalHandler::getSingleton().registerCallback(SIGINT,
                            std::bind( &SimFileConverter::callbackAbort, this), "SimFileConverter");
        ApplicationSignalHandler::getSingleton().registerCallback(SIGUSR2,
                            std::bind( &SimFileConverter::callbackAbort, this), "SimFileConverter");

        StateListType states;

        if(!m_simFile.openRead(f,true)){
            ERRORMSG("Could not open SimFile at :" << f)
        }else{
            LOG(m_log, "---> SimFile Properties:" <<std::endl << m_simFile.getDetails().getString() << std::endl)
        }

        // If we have a sim file info node, set the output
        EXPAND_PARAMETERPACK( details::StepperDispatch<TSimFileStepper>::initSimInfo(simFileStepper,
                                                                                     f,
                                                                                     perFileOutputFile,
                                                                                     m_simFile.getNSimBodies(),
                                                                                     m_simFile.getNStates()) )

        LOG(m_log, "---> Init sim info with: \n\toutputFile: " << perFileOutputFile << std::endl;)


        CPUTimer timer;
        timer.start();

        double start = 0, avgInitFrameTime = 0, avgStateTime = 0, avgStateLoadTime = 0;
        m_bodyCounter = 0;

        StateIdxType currentStateIdx = startIdx;
        auto itStateIdx = stateIndices.begin();
        // if state indices are given, overwrite start!
        if( !stateIndices.empty() && itStateIdx->m_idx > 0 ){
            currentStateIdx = itStateIdx->m_idx;
        }
        // set mappedIdx to start
        StateIdxType mappedIdx = mappedStartIdx;

        // Jump at beginning of first state
        m_simFile.seekgStates(currentStateIdx);

        // Set abort flag
        m_abort = (stateIndices.empty() && currentStateIdx >= endIdx );

        while(m_simFile.isGood() && !m_abort){

            boost::filesystem::path outputFile = "";

            // produce output for this state

            // load state
            double time;
            start = timer.elapsedMilliSec();
            m_simFile.read(states,time);
            //TODO get here a body state forward iterator instead of reading all states

            avgStateLoadTime +=  timer.elapsedMilliSec() - start;

            if(states.size()==0){
                ERRORMSG("State size is zero!")
            }
            LOG(m_log, "---> Loaded state at t: " <<time << std::endl;)

            // set frame name and output dir
            // =======================================================================

            // set output file name for this state (if specified) =====================================================
            if( !stateIndices.empty() && !itStateIdx->m_outputFile.empty() ){
                // we have a file path given for this state, take this
                outputFile  = itStateIdx->m_outputFile;
            }
            // ==========================================================================================

            // overwrite mappedIdx (if we have a list set it from the list, other wise default)
            // mappedIdx is not the same as stateIdx because (we want a linear index over the files)
            // which is possible by setting a different mappedIdx for each state index.

            if( !stateIndices.empty()){
                mappedIdx = itStateIdx->m_mappedIdx;
            }

            LOG(m_log, "---> Init state with: \n\toutputFile: " << outputFile << "\nmappedIdx: " << mappedIdx << "\n\ttime: " << time << std::endl;)
            start = timer.elapsedMilliSec();
            EXPAND_PARAMETERPACK( details::StepperDispatch<TSimFileStepper>::initState(simFileStepper,outputFile, time, mappedIdx ) )
            avgInitFrameTime += timer.elapsedMilliSec() - start;


            start = timer.elapsedMilliSec();
            /** apply full state or body state loop  for every stepper type */
            EXPAND_PARAMETERPACK(  (details::FullStateOrBodyState<TSimFileStepper,TSettings::FullState>::apply(this,simFileStepper, states)) )
            avgStateTime += timer.elapsedMilliSec() - start;


            EXPAND_PARAMETERPACK( details::StepperDispatch<TSimFileStepper>::finalizeState(simFileStepper) )

            // skip to next stateIdx if we have indices ========================
            if(!stateIndices.empty()){
                if(++itStateIdx != stateIndices.end()){
                    if(itStateIdx->m_idx < 0 || itStateIdx->m_idx == currentStateIdx){
                        ERRORMSG("Negative or same as privious state idx: " << itStateIdx->m_idx << " in xml for file: " << f)
                    }
                    // skip difference in file
                    m_simFile.seekgStates(itStateIdx->m_idx - currentStateIdx -1);
                    currentStateIdx = itStateIdx->m_idx;
                }else{
                    m_abort = true;
                }
            }else{ // we have no indices, take range
                // skip difference in file
                m_simFile.seekgStates(increment - 1);
                currentStateIdx += increment;
                if(currentStateIdx >= endIdx){
                    m_abort = true;
                }
                mappedIdx += increment; // default increment for mapped idx
            }
            // ===================================================================


            ++m_stateCounter;

            stop = false;
            EXPAND_PARAMETERPACK( stop |= details::StepperDispatch<TSimFileStepper>::isStopFrameLoop(simFileStepper) )
            if(stop){
                LOG(m_log,"---> Stop frame loop set, frameCount: " << m_stateCounter << " -> exit" << std::endl;)
                m_terminatedByStepper=true;
                break;
            }
        }

        if(!stateIndices.empty() && m_terminatedByStepper==false){
            if(itStateIdx != stateIndices.end()){
                LOG(m_log, "---> Warning: Reading simfile: "
                    << f << " became invalid before all state inidices have been converted" << std::endl)
            }
        }

          LOG(m_log, "---> Converter Speed:" <<std::endl
            << "Avg. Load State : "   << (avgStateLoadTime / m_stateCounter) << " ms" <<std::endl
            << "Avg. Init State : "   << (avgInitFrameTime / m_stateCounter) << " ms" <<std::endl
            << "Avg. Time per State : " << (avgStateTime / m_stateCounter) << " ms" <<std::endl
            << "Avg. Time per Body: " << (avgStateTime / (m_bodyCounter)) << " ms" <<std::endl;)


        ApplicationSignalHandler::getSingleton().unregisterCallback(SIGINT,"SimFileConverter");

    }

};

#endif // LogicConverter_hpp


