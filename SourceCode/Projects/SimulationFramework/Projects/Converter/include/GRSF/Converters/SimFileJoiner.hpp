#ifndef SimFileJoiner_hpp
#define SimFileJoiner_hpp

#include <iostream>
#include <iomanip>
#include <set>
#include <map>
#include <vector>

#include <boost/filesystem.hpp>
#include "GRSF/Common/ProgressBarCL.hpp"
#include "GRSF/Dynamics/General/MultiBodySimFile.hpp"


#include "GRSF/Common/LogDefines.hpp"


class SimFileJoiner {

public:
    struct RangeAll {};
private:

    template<typename TYPE, typename TYPE2 = TYPE>
    struct ListOrRangeTypes {
        using ListType = std::set<TYPE>; ///< ordered list!
        static const unsigned int ListTypeIdx = 0;
        using RangeType =  std::pair<TYPE2,TYPE2>; ///< range start to end values
        static const unsigned int RangeTypeIdx = 1;
        using AllType = RangeAll;
        static const unsigned int AllTypeIdx = 2;
        using VariantType = boost::variant< ListType, RangeType, AllType > ; // Indexes into the variants are defined above!

    };

    /**
    * returns 0 for not in range or list,
    *         1 for in range or in list and
    *         2 for in range or in list and FINISHED (only for list)
    */
    template<typename TYPE, typename TYPE2 = TYPE>
    struct InListOrInRangeVisitor: public boost::static_visitor<char> {
        InListOrInRangeVisitor(TYPE t, unsigned int bodyCounter): m_t(t), m_bodyC(bodyCounter) {};
        char operator()(typename ListOrRangeTypes<TYPE, TYPE2>::ListType & l) const {
            if(l.find(m_t)!=l.end()) {
                if( m_bodyC+1 == l.size()){
                    //notfiy that we are finished
                    return 2;
                }
                return 1;
            }
            return 0;
        }
        char operator()(typename ListOrRangeTypes<TYPE, TYPE2>::RangeType & r) const {
            // passes ranges tests if somewhere -1 is detected!
            if( (r.first ==-1 || r.first <= m_t) &&  (r.second == -1 || m_t <= r.second) ) {
                return 1;
            }
            return 0;
        }
        char operator()(typename ListOrRangeTypes<TYPE, TYPE2>::AllType & r) const {}

        TYPE m_t;
        unsigned int m_bodyC;
    };

    using InListOrInRangeBodyVisitorType = InListOrInRangeVisitor<unsigned int , long long int> ;
public:

    using TypesTimeRange = ListOrRangeTypes<double>        ;
    using TypesBodyRange = ListOrRangeTypes<unsigned int, long long int>  ;  // long long int for the range, it can be -1 -1

private:

    struct SimFileJoinerAllVisitor : public boost::static_visitor<bool> {
        SimFileJoinerAllVisitor(TypesTimeRange::VariantType & var1, TypesBodyRange::VariantType & var2):
            m_var1(var1), m_var2(var2)
        {

        }
        template<typename T1, typename T2>
        bool operator()(T1 & t1, T2 & t2) const {
            // we call joinAll()
            return false;
        } ;

        template<typename T1>
        bool operator()(T1 & t1, RangeAll & t2) const {
            // we call join(), but no body range specified, set it to all [-1,-1]
            std::cout << "---> no bodyrange given: setting to all range" << std::endl;
            m_var2 = TypesBodyRange::RangeType(-1,-1);
            return false;
        };

        template<typename T2>
        bool operator()( RangeAll & t1, T2 & t2) const {
            // we call join(), but no body range specified, set it to all [-1,-1]
            std::cout << "---> no timerange given: setting to all range" << std::endl;
            m_var1 = TypesTimeRange::RangeType(-1,-1);
            return false;
        };

        bool operator()(RangeAll & t1, RangeAll & t2) const {
            // we call join(), but no time range specified, set it to all [-1,-1]
            return true;
        };

        TypesTimeRange::VariantType & m_var1;
        TypesBodyRange::VariantType & m_var2;
    };

public:


    void join(const std::vector<boost::filesystem::path> & inputFiles,
              boost::filesystem::path outputFile,
              const TypesTimeRange::VariantType & timeRange ,
              const TypesBodyRange::VariantType & bodyRange ) {

        m_iFiles = inputFiles;
        m_oFile  = outputFile;
        m_timeRange = timeRange;
        m_bodyRange = bodyRange;

        SimFileJoinerAllVisitor all_vis(m_timeRange,m_bodyRange);

        if( boost::apply_visitor(all_vis, m_timeRange, m_bodyRange ) == true) {
            joinAll();
        } else {
            join();
        }

    }

private:

    std::vector<boost::filesystem::path>  m_iFiles;
    boost::filesystem::path m_oFile;

    TypesTimeRange::VariantType  m_timeRange;
    TypesBodyRange::VariantType  m_bodyRange;

    void joinAll() {
        std::cerr << "---> Execute Join (all)" << std::endl;
        // First open all files and check consistency!
        std::cerr << "---> Check consistency of input files..." << std::endl;
        unsigned int bodies, dofq, dofu;

        MultiBodySimFile simFile;
        std::streamsize bState;
        unsigned int i = 0;

        if(m_iFiles.size() <1) {
            ERRORMSG("To little input files specified!")
        }

        for(auto it=m_iFiles.begin(); it!=m_iFiles.end(); it++) {

            if(*it == m_oFile){
                ERRORMSG("Input/Output Files are the same!")
            }
            if(!simFile.openRead(*it)) {
                ERRORMSG(simFile.getErrorString());
            };
            if(i == 0) {
                dofq = simFile.getNDOFq();
                dofu = simFile.getNDOFu();
                bodies = simFile.getNSimBodies();
                bState = simFile.getBytesPerState();
            }
            if(i != 0 && (simFile.getNSimBodies() != bodies || simFile.getBytesPerState() != bState )) {
                ERRORMSG("Number of bodies: " << simFile.getNSimBodies() << " , bytesPerState: "
                               << simFile.getBytesPerState() << " of file: "
                               << *it << " do not match bodies: " << bodies << " , bytesPerState: "
                               << bState <<" of first file!");
            }
            std::cerr << simFile.getDetails().getString() << std::endl;
            simFile.close();
            i++;
        }

        // Join all states together
        std::cerr << "---> Open new output file at: "  <<  m_oFile << " bodies: " << bodies<< std::endl;
        MultiBodySimFile output;
        if(!output.openWrite_impl(m_oFile,
                                  dofq,
                                  dofu,
                                  bodies,
                                  true,
                                  simFile.m_additionalBytesPerBodyType,
                                  simFile.m_nAdditionalBytesPerBody))
        {
            ERRORMSG(output.getErrorString());
        };
        // Push all simfiles to output
        for(auto it=m_iFiles.begin(); it!=m_iFiles.end(); it++) {
            if(!simFile.openRead(*it)) {
                ERRORMSG(simFile.getErrorString());
            };
            output << simFile;
        }

        output.close();

    }

    void join() {
        std::cerr << "---> Execute Join (general)" << std::endl;
        // First open all files and check consistency!
        std::cerr << "---> Check consistency of input files..." << std::endl;
        unsigned int bodies, dofq, dofu;

        MultiBodySimFile simFile;
        std::streamsize bState;
        unsigned int i = 0;

        if(m_iFiles.size() <1) {
            ERRORMSG("To little input files specified!")
        }

        for(auto it=m_iFiles.begin(); it!=m_iFiles.end(); it++) {
            if(!simFile.openRead(*it)) {
                ERRORMSG(simFile.getErrorString());
            };
            if(i == 0) {
                dofq = simFile.getNDOFq();
                dofu = simFile.getNDOFu();
                bodies = simFile.getNSimBodies(); // May bee not important
                bState = simFile.getBytesPerState();
            }
            if(i != 0 && (simFile.getNSimBodies() != bodies || simFile.getBytesPerState() != bState )) {
                ERRORMSG("Number of bodies: " << simFile.getNSimBodies() << " , bytesPerState: "
                               << simFile.getBytesPerState() << " of file: "
                               << *it << " do not match bodies: " << bodies << " , bytesPerState: "
                               << bState <<" of first file!");
            }

            std::cerr << simFile.getDetails().getString() << std::endl;
            simFile.close();
            i++;
        }



        // Join states together
        std::cerr << "---> Open new output file at: "  <<  m_oFile << std::endl;
        MultiBodySimFile output;
        if(!output.openWrite_impl(m_oFile,dofq,dofu,0,true)){
            ERRORMSG(output.getErrorString());
        };

        // Push all simfiles to output

        bool firstFile = true;

        if( m_timeRange.which() == TypesTimeRange::ListTypeIdx) {

            auto & timeList =  boost::get<typename TypesTimeRange::ListType>(m_timeRange);

            auto currentMatchedTime = timeList.begin();
            for(auto it=m_iFiles.begin(); it!=m_iFiles.end(); it++) {
                if(!simFile.openRead(*it)) {
                    ERRORMSG(simFile.getErrorString());
                };

                std::cerr << "---> Process File: " << *it << std::endl;

                writeTo(output,simFile, timeList, currentMatchedTime, m_bodyRange,firstFile);


                if(firstFile){ firstFile=false;}
            }
        }
        else if( m_timeRange.which() == TypesTimeRange::RangeTypeIdx) {

            auto & timeRange =  boost::get<typename TypesTimeRange::RangeType>(m_timeRange);

            double currentWrittenTime = timeRange.first;

            for(auto it=m_iFiles.begin(); it!=m_iFiles.end(); it++) {
                if(!simFile.openRead(*it)) {
                    ERRORMSG(simFile.getErrorString());
                };

                std::cerr << "---> Process File: " << *it << std::endl;
                writeTo(output,simFile,timeRange, currentWrittenTime,  m_bodyRange, firstFile);

                if(firstFile){ firstFile=false;}
            }
        }

        simFile.close();
        output.close();

    }


    void writeTo(MultiBodySimFile & toFile,
                 MultiBodySimFile & fromFile,
                 TypesTimeRange::ListType & timeList,
                 TypesTimeRange::ListType::iterator & lastMatchedTime,
                 TypesBodyRange::VariantType & bodyRange,
                 bool firstFile = true) {
        std::cerr << std::setprecision(9) ;

        if( !toFile.m_file_stream.good() || !fromFile.m_file_stream.good()) {
            ERRORMSG("Some filestreams are not valid!")
        }

        if( lastMatchedTime == timeList.end()){
            std::cerr << "---> Already all matched! " << std::endl;
            return;
        }

        //loop over all times if time is in the range, extract all body ids and add to the
        fromFile.m_file_stream.seekg(fromFile.m_beginOfStates);


        // Scan all times and sort:
        std::cerr << "---> Scan Time ... " << std::endl;
        double currentTime;
        using TimeOffsetType = std::pair<double,std::streamoff>;
        std::vector< TimeOffsetType > fileTimesAndOff;
        for( std::streamoff stateIdx = 0; stateIdx < fromFile.m_nStates; stateIdx++) {
             fromFile >> currentTime;
             fileTimesAndOff.push_back(std::make_pair(currentTime, fromFile.m_file_stream.tellg()));
             // Jump over all bodies
             fromFile.m_file_stream.seekg( fromFile.m_nBytesPerState - std::streamoff(sizeof(double)) ,std::ios_base::cur);
        }
        std::sort(fileTimesAndOff.begin(),fileTimesAndOff.end(),
                  [](const TimeOffsetType & lhs, const TimeOffsetType & rhs) {
                     return lhs.first < rhs.first; });
        // Scan finished
        std::cerr << "---> Scan finished!" << std::endl;
        std::cerr << "---> TimeRange File: ["
        << fileTimesAndOff.begin()->first << "," << fileTimesAndOff.rbegin()->first << "]" << std::endl;

//        std::cerr << "Time scan finished: " << std::endl;
//        for( auto & e : fileTimesAndOff){
//            std::cerr << e.first << " ";
//        }
//        std::cerr << std::endl;

        fromFile.m_file_stream.seekg(fromFile.m_beginOfStates);

        if( fileTimesAndOff.size() == 0){
            ERRORMSG("No times in file found!")
        }

        // Match the timeList list
        std::vector< TimeOffsetType > matchedTimesAndRelOff; // Offset relative to last time, first time has offset relative to m_beginOfStates !
        matchedTimesAndRelOff.reserve(timeList.size());

        auto currMatchListTime = lastMatchedTime;
//        std::streamoff lastMatchOff = fromFile.m_file_stream.tellg();
        auto itFileTime = fileTimesAndOff.begin();

        if( *timeList.rbegin() < itFileTime->first ){ //max range time < smallest filetime
            std::cerr << "---> Nothing to match for this file: maxtime in timeList: " << *timeList.rbegin()
            << " below smallest fileTime: "<< itFileTime->first<< std::endl;
            return;
        }else{
            while(currMatchListTime != timeList.end() && itFileTime != fileTimesAndOff.end() ){

                if(  itFileTime->first < *currMatchListTime){
                      itFileTime++; // move file times
                }
                else if( (*currMatchListTime == itFileTime->first)  // if timeList and fileTime match exactly
                        ||
                    (std::next(currMatchListTime) == timeList.end() && *currMatchListTime < itFileTime->first )  //or if we are at the last listTime and fileTime is bigger
                        ||
                    (std::next(currMatchListTime) !=  timeList.end() && *currMatchListTime < itFileTime->first
                     && itFileTime->first < *std::next(currMatchListTime) ) // // itFileTime->first im intervall (currMatchListTime, currMatchListTime++)
                    ){
                        // match found
                        std::cerr << "---> Matched: rangeTime: " << *currMatchListTime<< "\t\t <--- \t\t" << itFileTime->first << " : fileTime " << std::endl;
                        matchedTimesAndRelOff.push_back(std::make_pair(itFileTime->first, itFileTime->second ));

                        itFileTime++;
                        currMatchListTime++;
                }else{
                    // moves rangeTime as long as the next filetime is not in the intervall
                    currMatchListTime++;
                }

//                std::cout << "List time: "<< *currMatchListTime << std::endl;
//                std::cout << "File time: "<< itFileTime->first << std::endl;
            }
        }

        if(matchedTimesAndRelOff.size()==0){
           std::cerr << "---> No times matched!" << std::endl;
           return;
        }

        std::streamsize size = fromFile.m_nBytesPerBody - std::streamsize(sizeof(RigidBodyIdType));
        std::vector<char> byteBuffer(size); // buffer
        unsigned int initBodyCounter = 0; bool firstRun = true;


        // Jump in file over the matched times and
        for( auto matchedTimesIt = matchedTimesAndRelOff.begin();
             matchedTimesIt != matchedTimesAndRelOff.end(); matchedTimesIt++){

            //Offset from current to next time! (time already read)
            fromFile.m_file_stream.seekg(matchedTimesIt->second);

            std::cerr << "---> Extract  bodies for time: " << matchedTimesIt->first << std::endl;
            toFile << matchedTimesIt->first;

            //Move over all bodies
            RigidBodyIdType id;
            unsigned int bodyCounter = 0;
            LOGSJ(std::cerr <<  "---> Write bodies: ";)
            for(unsigned int body = 0; body < fromFile.m_nSimBodies; body++) {
                fromFile >> id;

                //std::cerr << RigidBodyId::getBodyIdString(id) << ",";
                auto res = boost::apply_visitor(InListOrInRangeBodyVisitorType(RigidBodyId::getBodyNr(id), bodyCounter), bodyRange );
                if(res > 0 ) {
                    LOGSJ(std::cerr << RigidBodyId::getBodyIdString(id); );
                    bodyCounter++;
                    fromFile.m_file_stream.read(&byteBuffer[0],size);
                    toFile << id;
                    toFile.m_file_stream.write(&byteBuffer[0],size);

                    // Enough body added!
                    if( res == 2){
                        LOGSJ(std::cerr << std::endl << "---> Enough body found, break;" << std::endl;);
                        break;
                    }
                } else {
                    // State not found
                    // Jump body (u and q)
                    fromFile.m_file_stream.seekg( size ,std::ios_base::cur);
                }
            }
            LOGSJ(std::cerr <<std::endl;)

            if(!firstRun){
                if(bodyCounter != initBodyCounter){
                     ERRORMSG("At time :" << currentTime << " only " <<
                                     bodyCounter << " bodies found, instead of " << initBodyCounter << " as in first found state!")
                }
            }else{
                initBodyCounter = bodyCounter;
            }
        }


        // Rewrite header if first file
        if(firstFile){
            toFile.m_additionalBytesPerBodyType = fromFile.m_additionalBytesPerBodyType;
            toFile.m_nAdditionalBytesPerBody = fromFile.m_nAdditionalBytesPerBody;
            toFile.m_nSimBodies = initBodyCounter;
            toFile.writeHeader();
        }

        // Set latest matched time;
        lastMatchedTime = currMatchListTime;
    }

    void writeTo(MultiBodySimFile & toFile,
                 MultiBodySimFile & fromFile,
                 TypesTimeRange::RangeType & timeRange,
                 double & lastTimeWritten,
                 TypesBodyRange::VariantType & bodyRange,
                 bool firstFile = true) {




        if( !toFile.m_file_stream.good() || !fromFile.m_file_stream.good()) {
            ERRORMSG("Some filestreams are not valid!")
        }

        //loop over all times if time is in the range, extract all body ids and add to the
        fromFile.m_file_stream.seekg(fromFile.m_beginOfStates);


        // Scan all times and sort:
        std::cerr << "---> Scan Time ... " << std::endl;
        double currentTime;
        using TimeOffsetType = std::pair<double,std::streamoff>;

        std::vector< TimeOffsetType > fileTimesAndOff;
        for( std::streamoff stateIdx = 0; stateIdx < fromFile.m_nStates; stateIdx++) {
             fromFile >> currentTime;
             fileTimesAndOff.push_back(std::make_pair(currentTime, fromFile.m_file_stream.tellg()));
             // Jump over all bodies
             fromFile.m_file_stream.seekg( fromFile.m_nBytesPerState - std::streamoff(sizeof(double)) ,std::ios_base::cur);
        }


        std::sort(fileTimesAndOff.begin(),fileTimesAndOff.end(),
                  [](const TimeOffsetType & lhs, const TimeOffsetType & rhs) { return lhs.first < rhs.first; }
                  );
        // Scan finished
        std::cerr << "---> Scan finished!" << std::endl;
        std::cerr << "---> TimeRange File: ["
        << fileTimesAndOff.begin()->first << "," << fileTimesAndOff.rbegin()->first << "]" << std::endl;

        // If intersection of range does not include the files Time range break
        if( (timeRange.second!=-1 && fileTimesAndOff.begin()->first > timeRange.second ) ||
            (timeRange.first!=-1 && timeRange.first >  fileTimesAndOff.rbegin()->first ) ){
            std::cout << "---> timeRange out of range: return" << std::endl;
            return;
        }

        if( lastTimeWritten > fileTimesAndOff.rbegin()->first ){ // max fileTime needs to be greater then lastTimeWritten
            ERRORMSG("last time written: " << lastTimeWritten << " is greater then max file time: " << fileTimesAndOff.rbegin()->first)
        }

        // get startit (gibt nicht end zurück, da end file Time grösser als lastTimeWritten)
        auto startIt = fileTimesAndOff.begin();
        if(!firstFile){
            startIt = std::upper_bound( fileTimesAndOff.begin(), fileTimesAndOff.end(), lastTimeWritten ,
                                            []( const double & val , const TimeOffsetType & lhs ) { return  val < lhs.first ;});
            ASSERTMSG(startIt != fileTimesAndOff.end(),"END iterator");
        }

        if( !firstFile && startIt->first == lastTimeWritten){
                std::cout << "---> Time: " << startIt->first << " already written, break" << std::endl;
                return;
        }

        std::streamsize size = fromFile.m_nBytesPerBody - std::streamsize(sizeof(RigidBodyIdType));
        std::vector<char> byteBuffer(size); // buffer 1mb of bodies
        unsigned int initBodyCounter = 0; bool firstRun = true;



        // Jump in file over the matched times and
        for( auto fileTimeIt = startIt; fileTimeIt != fileTimesAndOff.end(); fileTimeIt++){

            if( timeRange.second != -1 && fileTimeIt->first > timeRange.second  ){
                break;
            }

            //Offset from current to next time! (time already read)
            fromFile.m_file_stream.seekg(fileTimeIt->second);

            std::cerr << "---> Extract bodies for time: " << fileTimeIt->first << std::endl;
            // Set latest written time for next files
            lastTimeWritten = fileTimeIt->first;
            // Write time
            toFile << fileTimeIt->first;

            //Move over all bodies
            RigidBodyIdType id;
            unsigned int bodyCounter = 0;
            LOGSJ(std::cerr <<  "---> Write bodies: ";)
            for(unsigned int body = 0; body < fromFile.m_nSimBodies; body++) {
                fromFile >> id;

                //std::cerr << RigidBodyId::getBodyIdString(id) << ",";
                auto res = boost::apply_visitor(InListOrInRangeBodyVisitorType(RigidBodyId::getBodyNr(id), bodyCounter), bodyRange );
                if(res > 0 ) {
                    LOGSJ(std::cerr << RigidBodyId::getBodyIdString(id) << ", ";)
                    bodyCounter++;
                    fromFile.m_file_stream.read(&byteBuffer[0],size);
                    toFile << id;
                    toFile.m_file_stream.write(&byteBuffer[0],size);

                    // Enough body added!
                    if( res == 2){
                        LOGSJ(std::cerr <<std::endl<< "---> Enough body found, break;" << std::endl;)
                        break;
                    }
                } else {
                    // State not found
                    // Jump body (u and q)
                    fromFile.m_file_stream.seekg( size ,std::ios_base::cur);
                }
            }
            LOGSJ(std::cerr <<std::endl;)

            if(!firstRun){
                if(bodyCounter != initBodyCounter){
                     ERRORMSG("At time :" << currentTime << " only " <<
                                     bodyCounter << " bodies found, instead of " << initBodyCounter << " as in first found state!")
                }
            }else{
                initBodyCounter = bodyCounter;
            }
        }


        // Rewrite header if first file
        if(firstFile){
            toFile.m_additionalBytesPerBodyType = fromFile.m_additionalBytesPerBodyType;
            toFile.m_nAdditionalBytesPerBody = fromFile.m_nAdditionalBytesPerBody;
            toFile.m_nSimBodies = initBodyCounter;
            toFile.writeHeader();
        }

    }
};

#endif // SimFileJoiner_hpp


