#ifndef LogicConverter_hpp
#define LogicConverter_hpp

#include <iostream>
#include <iomanip>
#include <set>
#include <map>
#include <string>
#include <vector>

#include <pugixml.hpp>

#include <boost/filesystem.hpp>

#include "GRSF/Common/LogDefines.hpp"
#include "GRSF/Common/TypeDefs.hpp"

#include "GRSF/Common/SimpleLogger.hpp"
#include "GRSF/Dynamics/General/MultiBodySimFile.hpp"

#include "GRSF/Common/ApplicationSignalHandler.hpp"

#include "GRSF/Common/ContainerTag.hpp"
#include "GRSF/Common/SfinaeMacros.hpp"
#include "GRSF/Common/HasMemberFunction.hpp"
#include "GRSF/Common/ExpandParameterPack.hpp"

#include "GRSF/Common/CPUTimer.hpp"
#include "GRSF/Common/CommonFunctions.hpp"
#include "GRSF/Common/ProgressBarCL.hpp"

//#include "GRSF/Logic/DummyNode.hpp"


class SimFileConverter {
public:

    using XMLNodeType = pugi::xml_node;
    using XMLNodeItType = pugi::xml_node_iterator;
    using XMLAttributeType = pugi::xml_attribute;

    SimFileConverter(const std::vector<boost::filesystem::path> & inputFiles,
                  boost::filesystem::path outputFile,
                  boost::filesystem::path outputDir){

        m_outputFile = outputFile;
        m_outputDir  = m_outputDir;
        m_inputFiles = inputFiles;

        auto log = outputFile.parent_path() / "LogicConverter.log";
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
        bool stop;
        LOG(m_log, "---> LogicConverter started:" <<std::endl;);

        // global framecounter
        m_frameCounter = 0;

        // First open the sim file (if .sim extension)
        // if .xml extension (then this is the process file where each simfile and frame index is stored)

        unsigned int fileIdx = 0;
        for(auto file : m_inputFiles) {


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

                    stateIndices.clear();
                    std::string uuid = n.attribute("uuid").value();
                    boost::filesystem::path path = n.attribute("simFile").value();

                    if(path.empty()){
                        LOG(m_log,"---> No simFile path given, skip this file!" << std::endl;)
                        continue;
                    }

                    // parse frame index list (assumed to be sorted! otherwise exception in convertFile)
                    StateIdxType idx;
                    unsigned int frameIdx;
                    boost::filesystem::path outputFile;
                    for(auto s : n.children("State")){

                        if( !Utilities::stringToType(idx, s.attribute("stateIdx").value() )  ) {
                                ERRORMSG("---> String conversion to obtain state id failed!");
                        }

                        if( !Utilities::stringToType(frameIdx, s.attribute("frameIdx").value() )  ) {
                                ERRORMSG("---> String conversion to obtain state id failed!");
                        }

                        outputFile = "";
                        auto att = s.attribute("outputFile");
                        if( att ) {
                           outputFile = att.value();
                        }

                        // add to list
                        stateIndices.push_back( StateIndex{idx,frameIdx,outputFile});
                    }

                    LOG(m_log,"---> Parsed " << stateIndices.size() << " state for file: " << path << "from XML: " << file.filename() << std::endl;)

                    if( stateIndices.size() > 0){
                        convertFile<TSettings,TSimFileStepper...>(path,uuid,stateIndices,simFileStepper...);
                    }else{
                        LOG(m_log,"---> No states to process..." << std::endl;)
                    }



                }

            }else{
                // try to convert sim file
                convertFile<TSettings, TSimFileStepper...>(file, std::to_string(fileIdx) , {},  simFileStepper...);
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

    using StateIdxType = std::streamoff;
    struct StateIndex{
        StateIdxType m_idx;
        unsigned int m_frameIdx;
        boost::filesystem::path m_outputFile;
    };

    using StateIndicesType = std::vector< StateIndex >;
    using StateListType = std::vector<RigidBodyStateAdd>;

    MultiBodySimFile m_simFile;

    Logging::Log * m_log;

    boost::filesystem::path m_outputFile;
    boost::filesystem::path m_outputDir;

    std::vector<boost::filesystem::path> m_inputFiles;

    std::size_t m_frameCounter; ///< Counting all frames converted
    std::size_t m_bodyCounter;  ///< Counting all bodies converted

    bool m_terminatedByStepper;

    bool m_abort;
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
            SIMFILESTEPPER_DISPATCH_FUNC(initFrame)
            SIMFILESTEPPER_DISPATCH_FUNC(addBodyState)
            SIMFILESTEPPER_DISPATCH_FUNC(addState)
            SIMFILESTEPPER_DISPATCH_FUNC(finalizeFrame)

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
            SIMFILESTEPPER_DISPATCH_FUNC_CONT(initFrame)
            SIMFILESTEPPER_DISPATCH_FUNC_CONT(addBodyState)
            SIMFILESTEPPER_DISPATCH_FUNC_CONT(addState)
            SIMFILESTEPPER_DISPATCH_FUNC_CONT(finalizeFrame)

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
                     TSimFileStepper&&... simFileStepper
                    )
    {
        bool stop = false;
        LOG(m_log, "---> Converting file:" << f << std::endl;);

        m_abort = false;
        ApplicationSignalHandler::getSingleton().registerCallback(SIGINT,
                            std::bind( &SimFileConverter::callbackAbort, this), "SimFileConverter");

        StateListType states;

        if(!m_simFile.openRead(f,true)){
            ERRORMSG("Could not open SimFile at :" << f)
        }else{
            LOG(m_log, "---> SimFile Properties:" <<std::endl << m_simFile.getDetails().getString() << std::endl)
        }

        // If we have a sim file info node, set the output
        EXPAND_PARAMETERPACK( details::StepperDispatch<TSimFileStepper>::initSimInfo(simFileStepper, m_simFile.getNSimBodies(),m_simFile.getNStates()) )


        CPUTimer timer;
        timer.start();

        double start = 0, avgInitFrameTime = 0, avgStateTime = 0, avgStateLoadTime = 0;
        m_bodyCounter = 0;

        StateIdxType currentStateIdx = 0;
        auto itStateIdx = stateIndices.begin();
        // Jump at beginning of first state
        if( !stateIndices.empty() && itStateIdx->m_idx > 0 ){
            currentStateIdx = itStateIdx->m_idx;
            m_simFile.seekgStates(currentStateIdx);
        }


        while(m_simFile.isGood() && !m_abort){

            std::string outputName;
            boost::filesystem::path outputDir = "./";

            // produce render output file for this state

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
            // if function argument is given take this
            if(!m_outputDir.empty()){
                outputDir  = m_outputDir;
            }
            // set output file name for this state,
            if( m_outputFile.empty() && !stateIndices.empty() && !itStateIdx->m_outputFile.empty() ){
                // we have a file path given and no function argument, take from state
                if( !itStateIdx->m_outputFile.has_filename()){
                    ERRORMSG(" State idx: " << itStateIdx->m_idx << " has no correct filename!")
                }
                outputDir  /= itStateIdx->m_outputFile.parent_path();
                outputName = itStateIdx->m_outputFile.filename().string();
            }else{
                // format default file name otherwise (if there is no file path in xml or function argument is given)
                std::string baseFilename =  m_outputFile.filename().string();
                if( baseFilename.empty()){
                    baseFilename = "Frame";
                }
                outputDir  /= m_outputFile.parent_path();
                outputName = baseFilename +"-id-"+uuidString + "-s-" + std::to_string(m_frameCounter);
            }

            // set frame idx (if we have a list set it from the list, other wise default)
            unsigned int frameIdx = m_frameCounter;
            if( !stateIndices.empty()){
                frameIdx = itStateIdx->m_frameIdx;
            }

            LOG(m_log, "---> Init frame with: \n\toutputDir: " << outputDir
                << "\n\tframeName: " << outputName << "\n\tframeIdx: " << frameIdx << "\n\ttime: " << time << std::endl;)

            start = timer.elapsedMilliSec();
            EXPAND_PARAMETERPACK( details::StepperDispatch<TSimFileStepper>::initFrame(simFileStepper,outputDir, outputName , time, frameIdx ) )
            avgInitFrameTime += timer.elapsedMilliSec() - start;


            start = timer.elapsedMilliSec();
            /** apply full state or body state loop  for every stepper type */
            EXPAND_PARAMETERPACK(  (details::FullStateOrBodyState<TSimFileStepper,TSettings::FullState>::apply(this,simFileStepper, states)) )

            avgStateTime += timer.elapsedMilliSec() - start;


            EXPAND_PARAMETERPACK( details::StepperDispatch<TSimFileStepper>::finalizeFrame(simFileStepper) )

            // skip to next stateIdx if we have indices
            if(!stateIndices.empty()){
                if(++itStateIdx != stateIndices.end()){
                    if(itStateIdx->m_idx < 0 || itStateIdx->m_idx == currentStateIdx){
                        ERRORMSG("Negative or same as privious state idx: " << itStateIdx->m_idx << " in xml for file: " << f)
                    }
                    // skip difference
                    m_simFile.seekgStates(itStateIdx->m_idx - currentStateIdx -1);
                    currentStateIdx = itStateIdx->m_idx;
                }else{
                    m_abort = true;
                }
            }else{
                //otherwise dont skip, but update stateIdx
                ++currentStateIdx;
            }


            m_frameCounter++;

            stop = false;
            EXPAND_PARAMETERPACK( stop |= details::StepperDispatch<TSimFileStepper>::isStopFrameLoop(simFileStepper) )
            if(stop){
                LOG(m_log,"---> Stop frame loop set, frameCount: " << m_frameCounter << " -> exit" << std::endl;)
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
            << "Avg. Load State : "   << (avgStateLoadTime / m_frameCounter) << " ms" <<std::endl
            << "Avg. Init Frame : "   << (avgInitFrameTime / m_frameCounter) << " ms" <<std::endl
            << "Avg. Loop Time  / Body: " << (avgStateTime / (m_bodyCounter)) << " ms" <<std::endl
            << "Avg. Loop Time / Frame : " << (avgStateTime / m_frameCounter) << " ms" <<std::endl;)

        ApplicationSignalHandler::getSingleton().unregisterCallback(SIGINT,"SimFileConverter");

    }

};

#endif // LogicConverter_hpp


