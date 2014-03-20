#ifndef SimFileJoiner_hpp
#define SimFileJoiner_hpp


#include <boost/filesystem.hpp>
#include <boost/variant.hpp>
#include "MultiBodySimFile.hpp"


class SimFileJoiner {

public:
    struct RangeAll {};
private:
    template<typename TYPE>
    struct ListOrRangeTypes {
        using ListType = std::set<TYPE>; ///< ordered list!
        static const unsigned int ListTypeIdx = 0;
        using RangeType =  std::pair<TYPE,TYPE>; ///< range start to end values
        static const unsigned int RangeTypeIdx = 1;
        using AllType = RangeAll;
        static const unsigned int AllTypeIdx = 2;
        using VariantType = boost::variant< ListType, RangeType, AllType > ;

    };

    template<typename TYPE>
    struct InListOrInRangeVisitor: public boost::static_visitor<bool> {
        InListOrInRangeVisitor(TYPE t): m_t(t) {};
        bool operator()(typename ListOrRangeTypes<TYPE>::ListType & l) const {
            if(l.find(m_t)!=l.end()) {
                return true;
            }
            return false;
        }
        bool operator()(typename ListOrRangeTypes<TYPE>::RangeType & r) const {
            if(r.first <= m_t &&  m_t <= r.second) {
                return true;
            }
            return false;
        }
        bool operator()(typename ListOrRangeTypes<TYPE>::AllType & r) const {}

        TYPE m_t;
    };

    using InListOrInRangeBodyVisitorType = InListOrInRangeVisitor<unsigned int> ;

    struct SimFileJoinerAllVisitor : public boost::static_visitor<bool> {
        template<typename T1, typename T2>
        bool operator()(T1 & t1, T2 & t2) const {return false;} ;
        bool operator()(RangeAll & t1, RangeAll & t2) const {return true;};
    };

public:

    typedef ListOrRangeTypes<double>         TypesTimeRange;
    typedef ListOrRangeTypes<unsigned int>   TypesBodyRange;



    void join(const std::vector<boost::filesystem::path> & inputFiles,
              boost::filesystem::path outputFile,
              const TypesTimeRange::VariantType & timeRange ,
              const TypesBodyRange::VariantType & bodyRange ) {

        m_iFiles = inputFiles;
        m_oFile  = outputFile;
        m_timeRange = timeRange;
        m_bodyRange = bodyRange;

        SimFileJoinerAllVisitor all_vis;

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

    /** only used for ListType for times */
    std::set<double>  m_bkWrittenTime; ///<Bookkeeping of written time, dont write same time twice!
    std::set<double>  m_bkMatchedTime; ///<Bookkeeping of written time, dont match two times to the same time in list

    void joinAll() {
        std::cerr << "---> Execute Join (all)" << std::endl;
        // First open all files and check consistency!
        std::cerr << "---> Check consistency of input files..." << std::endl;
        unsigned int bodies, dofq, dofu;

        MultiBodySimFile simFile;
        std::streamsize bState;
        unsigned int i = 0;

        if(m_iFiles.size() <1) {
            THROWEXCEPTION("To little input files specified!")
        }

        for(auto it=m_iFiles.begin(); it!=m_iFiles.end(); it++) {
            if(!simFile.openRead(*it)) {
                THROWEXCEPTION(simFile.getErrorString());
            };
            if(i == 0) {
                dofq = simFile.getNDOFq();
                dofu = simFile.getNDOFu();
                bodies = simFile.getNSimBodies();
                bState = simFile.getBytesPerState();
            }
            if(i != 0 && (simFile.getNSimBodies() != bodies || simFile.getBytesPerState() != bState )) {
                THROWEXCEPTION("Number of bodies: " << simFile.getNSimBodies() << " , bytesPerState: "
                               << simFile.getBytesPerState() << " of file: "
                               << *it << " do not match bodies: " << bodies << " , bytesPerState: "
                               << bState <<" of first file!");
            }

            simFile.close();
            i++;
        }

        // Join all states together
        std::cerr << "---> Open new output file at: "  <<  m_oFile << " bodies: " << bodies<< std::endl;
        MultiBodySimFile output;
        if(!output.openWrite_impl(m_oFile,dofq,dofu, bodies, simFile.m_additionalBytesType, simFile.m_nAdditionalBytesPerBody, true)){
            THROWEXCEPTION(output.getErrorString());
        };
        // Push all simfiles to output
        for(auto it=m_iFiles.begin(); it!=m_iFiles.end(); it++) {
            if(!simFile.openRead(*it)) {
                THROWEXCEPTION(simFile.getErrorString());
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
            THROWEXCEPTION("To little input files specified!")
        }

        for(auto it=m_iFiles.begin(); it!=m_iFiles.end(); it++) {
            if(!simFile.openRead(*it)) {
                THROWEXCEPTION(simFile.getErrorString());
            };
            if(i == 0) {
                dofq = simFile.getNDOFq();
                dofu = simFile.getNDOFu();
                bodies = simFile.getNSimBodies(); // May bee not important
                bState = simFile.getBytesPerState();
            }
            if(i != 0 && (simFile.getNSimBodies() != bodies || simFile.getBytesPerState() != bState )) {
                THROWEXCEPTION("Number of bodies: " << simFile.getNSimBodies() << " , bytesPerState: "
                               << simFile.getBytesPerState() << " of file: "
                               << *it << " do not match bodies: " << bodies << " , bytesPerState: "
                               << bState <<" of first file!");
            }

            simFile.close();
            i++;
        }

        // Join states together
        std::cerr << "---> Open new output file at: "  <<  m_oFile << std::endl;
        MultiBodySimFile output;
        if(!output.openWrite_impl(m_oFile,dofq,dofu, 0, 0, 0, true)){
            THROWEXCEPTION(output.getErrorString());
        };

        // Push all simfiles to output

        m_bkWrittenTime.clear();
        m_bkMatchedTime.clear();

        bool firstFile = true;
        for(auto it=m_iFiles.begin(); it!=m_iFiles.end(); it++) {
            if(!simFile.openRead(*it)) {
                THROWEXCEPTION(simFile.getErrorString());
            };
            std::cout << "---> Process File: " << *it << std::endl;
            if( m_timeRange.which() == TypesTimeRange::ListTypeIdx) {

                auto & l =  boost::get<typename TypesTimeRange::ListType>(m_timeRange);
                writeTo(output,simFile, l, m_bkWrittenTime, m_bkMatchedTime, m_bodyRange,firstFile);

            } else if( m_timeRange.which() == TypesTimeRange::RangeTypeIdx) {

                auto & r =  boost::get<typename TypesTimeRange::RangeType>(m_timeRange);
                writeTo(output,simFile,r, m_bkWrittenTime,  m_bodyRange, firstFile);
            }

            if(firstFile){ firstFile=false;}
        }

        std::cout << "---> Matched time: [ " ;
        std::copy(m_bkWrittenTime.begin(),m_bkWrittenTime.end(),std::ostream_iterator<double>(std::cout," "));
        std::cout << "]" << std::endl;


        simFile.close();
        output.close();

    }


    void writeTo(MultiBodySimFile & toFile,
                 MultiBodySimFile & fromFile,
                 TypesTimeRange::ListType & timeRange,
                 std::set<double> & bookWrittenTimes,
                 std::set<double> & bookMatchedTimes,
                 TypesBodyRange::VariantType & bodyRange,
                 bool firstFile = true) {


        if( !toFile.m_file_stream.good() || !fromFile.m_file_stream.good()) {
            THROWEXCEPTION("Some filestreams are not valid!")
        }

        std::set<RigidBodyIdType> bookBodyIds;

        //loop over all times if time is in the range, extract all body ids and add to the
        fromFile.m_file_stream.seekg(fromFile.m_beginOfStates);


        std::streamsize size = fromFile.m_nBytesPerBody - std::streamsize(sizeof(RigidBodyIdType));
        std::vector<char> byteBuffer(size); // buffer 1mb of bodies
        unsigned int initBodyCounter = 0; bool firstRun = true;

        auto hintIt = timeRange.begin();

        double currentTime;
        bool newState;
        for( std::streamoff stateIdx = 0; stateIdx < fromFile.m_nStates; stateIdx++) {

            newState = false;
            fromFile >> currentTime;

            //std::cout << currentTime << std::endl;
            if( *timeRange.begin() > currentTime) { // if currentTime is before range, break
                // Jump over all bodies
                fromFile.m_file_stream.seekg( fromFile.m_nBytesPerState - std::streamoff(sizeof(double)) ,std::ios_base::cur);
                continue;
            }


            double matchedTime;

            double endTime = *timeRange.rbegin();
            if(  endTime <= currentTime ){
                //std::cout << "Here1";
                matchedTime = endTime;
            }else if( std::next(hintIt) != timeRange.end() && *hintIt<= currentTime &&  currentTime < *std::next(hintIt) ) {
                //std::cout << "Here2";
                // check if hintIt is usefull;
                  matchedTime = *hintIt;

            }else{ //(currTime<last) lower_bound does not return end
                //std::cout << "Here3:"<< *hintIt;
                hintIt = timeRange.lower_bound(currentTime); // currentTime <= *lbTime ( no || end  becaus above!)
                if(*hintIt > currentTime) {
                    hintIt = std::prev(hintIt);
                }
                matchedTime = *hintIt;
            }


            // if the matched value is not already matched and the corresponding currentTIme has not yet been written
            if( bookMatchedTimes.insert(matchedTime).second && bookWrittenTimes.insert(currentTime).second){
                    // we inserted the value
                    newState = true;

            }else{
                //std::cout << "no write";
            }



            //New state has been found, move over all bodies and write the ones which are either in the range or in the list!
            if(newState) {
                // write time
                std::cout << "Write time: " << currentTime << std::endl;
                toFile << currentTime;

                //Move over all bodies
                RigidBodyIdType id;
                unsigned int bodyCounter = 0;
                for(unsigned int body = 0; body < fromFile.m_nSimBodies; body++) {
                    fromFile >> id;

                    //std::cout << RigidBodyId::getBodyIdString(id) << "," << std::endl;;
                    if( boost::apply_visitor(InListOrInRangeBodyVisitorType(RigidBodyId::getBodyNr(id)), bodyRange ) == true ) {

                        std::cout << "Write body: " << RigidBodyId::getBodyIdString(id) << std::endl;
                        bodyCounter++;
                        fromFile.m_file_stream.read(&byteBuffer[0],size);
                        toFile << id;
                        toFile.m_file_stream.write(&byteBuffer[0],size);

                    } else {
                        // State not found
                        // Jump body (u and q)
                        fromFile.m_file_stream.seekg( size ,std::ios_base::cur);
                    }
                }

                if(!firstRun){
                    if(bodyCounter != initBodyCounter){
                         THROWEXCEPTION("At time :" << currentTime << " only " <<
                                         bodyCounter << " bodies found, instead of " << initBodyCounter << " as in first found state!")
                    }
                }else{
                    initBodyCounter = bodyCounter;
                }

                // If we matched all times break
                if(bookMatchedTimes.size() == timeRange.size()){
                    std::cout <<" matched all" << std::endl;
                    break;
                }

            }else{
                // Jump over all bodies
                fromFile.m_file_stream.seekg( fromFile.m_nBytesPerState - std::streamoff(sizeof(double)) ,std::ios_base::cur);
            } // New state



        }


        // Rewrite header if first file
        if(firstFile){
            toFile.m_additionalBytesType = fromFile.m_additionalBytesType;
            toFile.m_nAdditionalBytesPerBody = fromFile.m_nAdditionalBytesPerBody;
            toFile.m_nSimBodies = initBodyCounter;
            toFile.writeHeader();
        }
    }

    void writeTo(MultiBodySimFile & toFile,
                 MultiBodySimFile & fromFile,
                 TypesTimeRange::RangeType & timeRange,
                 std::set<double> & bookWrittenTimes,
                 TypesBodyRange::VariantType & bodyRange,
                 bool firstFile = true) {

        if( !toFile.m_file_stream.good() || !fromFile.m_file_stream.good()) {
            ERRORMSG("Some filestreams are not valid!")
        }
        //loop over all times if time is in the range, extract all body ids and add to the
        fromFile.m_file_stream.seekg(fromFile.m_beginOfStates);

        double currentTime;
        bool newState;
        for( std::streamoff stateIdx = 0; stateIdx < fromFile.m_nStates; stateIdx++) {

            newState = false;
            fromFile >> currentTime;
            // Check if in range!
            if(timeRange.first <= currentTime && currentTime <= timeRange.second) {
                if( bookWrittenTimes.insert(currentTime).second == true) {
                    newState = true;
                    break;
                }
            }
        }


    }
};


#endif // SimFileJoiner_hpp


