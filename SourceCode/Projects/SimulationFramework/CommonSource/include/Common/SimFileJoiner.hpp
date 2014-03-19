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
        bool operator()(typename ListOrRangeTypes<TYPE>::ListType & l) {
            if(l.find(m_t)!=l.end()) {
                return true;
            }
            return false;
        }
        bool operator()(typename ListOrRangeTypes<TYPE>::RangeType & r) {
            if(r.first <= m_t &&  m_t <= r.second) {
                return true;
            }
            return false;
        }
        bool operator()(typename ListOrRangeTypes<TYPE>::AllType & r) {}

        TYPE m_t;
    };

    using InListOrInRangeBodyVisitorType = InListOrInRangeVisitor<unsigned int> ;

public:

    typedef ListOrRangeTypes<double>         TypesTimeRange;
    typedef ListOrRangeTypes<unsigned int>   TypesBodyRange;

    class SimFileJoinerAllVisitor : public boost::static_visitor<bool> {
    public:
        template<typename T1, typename T2>
        bool operator()(T1 & t1, T2 & t2) {
            return false;
        };

        bool operator()(RangeAll & t1,
                        RangeAll & t2) {
            return true;
        };
    };

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
        output.openWrite(m_oFile,dofq,dofu,bodies);

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

        // Join states together
        std::cerr << "---> Open new output file at: "  <<  m_oFile << " bodies: " << bodies<< std::endl;
        MultiBodySimFile output;
        output.openWrite(m_oFile,dofq,dofu,bodies);

        // Push all simfiles to output

        m_bkWrittenTime.clear();
        m_bkMatchedTime.clear();

        for(auto it=m_iFiles.begin(); it!=m_iFiles.end(); it++) {
            if(!simFile.openRead(*it)) {
                THROWEXCEPTION(simFile.getErrorString());
            };

            if( m_timeRange.which() == TypesTimeRange::ListTypeIdx) {

                auto & l =  boost::get<typename TypesTimeRange::ListType>(m_timeRange);
                writeTo(output,simFile, l, m_bkWrittenTime, m_bkMatchedTime, m_bodyRange);

            } else if( m_timeRange.which() == TypesTimeRange::RangeTypeIdx) {

                auto & r =  boost::get<typename TypesTimeRange::RangeType>(m_timeRange);
                writeTo(output,simFile,r, m_bkWrittenTime,  m_bodyRange);
            }
        }

        output.close();

    }


    void writeTo(MultiBodySimFile & toFile,
                 MultiBodySimFile & fromFile,
                 TypesTimeRange::ListType & timeRange,
                 std::set<double> & bookWrittenTimes,
                 std::set<double> & bookMatchedTimes,
                 TypesBodyRange::VariantType & bodyRange) {

        if( !toFile.m_file_stream.good() || !fromFile.m_file_stream.good()) {
            ERRORMSG("Some filestreams are not valid!")
        }
        //loop over all times if time is in the range, extract all body ids and add to the
        fromFile.m_file_stream.seekg(fromFile.m_beginOfStates);

        std::vector<char> byteBuffer(fromFile.m_nBytesPerBody - sizeof(RigidBodyIdType) );

        double currentTime;
        bool newState;
        for( std::streamoff stateIdx = 0; stateIdx < fromFile.m_nStates; stateIdx++) {

            newState = false;
            fromFile >> currentTime;

            // Check with list
            if( *(timeRange.begin()) <= currentTime) { // if currentTime is
                auto itj = timeRange.begin();
                for(auto it = timeRange.begin(); it!= timeRange.end(); it++) {
                    ++itj;
                    if(itj != timeRange.end()) {
                        if(  *it <= currentTime && currentTime < *itj ) {
                            // if the time has NOT already been written or the matched time was NOT already matched BEFORE
                            if(bookMatchedTimes.insert(*it).second  && bookWrittenTimes.insert(currentTime).second) {
                                newState = true;
                                break;
                            }

                        }
                    } else {
                        //Check last element
                        if(  *it <= currentTime  ) {
                            if( bookMatchedTimes.insert(*it).second  && bookWrittenTimes.insert(currentTime).second) {
                                newState = true;
                                break;
                            }
                        }
                    }
                }
            }

            //New state has been found, move over all bodies and write the ones which are either in the range or in the list!



            if(newState) {
                // write time


                //Move over all bodies
                RigidBodyIdType id;
                for(unsigned int body = 0; body < fromFile.m_nSimBodies; body++) {
                    fromFile >> id;

                    //std::cout << RigidBodyId::getBodyIdString(id) << std::endl;
                    InListOrInRangeBodyVisitorType vis(RigidBodyId::getBodyNr(id));
                    if( boost::apply_visitor(vis, bodyRange ) == true ) {

                        //write body


                    } else {
                        // State not found
                        // Jump body (u and q)
                        fromFile.m_file_stream.seekg( fromFile.m_nBytesPerBody - sizeof(id) ,std::ios_base::cur);
                    }
                }


            }

        }

    }

    void writeTo(MultiBodySimFile & toFile,
                 MultiBodySimFile & fromFile,
                 TypesTimeRange::RangeType & timeRange,
                 std::set<double> & bookWrittenTimes,
                 TypesBodyRange::VariantType & bodyRange) {

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


