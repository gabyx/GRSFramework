#ifndef SimFileInfo_hpp
#define SimFileInfo_hpp

#include <iostream>
#include <iomanip>
#include <set>
#include <map>
#include <vector>
#include <memory>

#include "pugixml.hpp"
#include <boost/filesystem.hpp>

#include "GRSF/Common/ProgressBarCL.hpp"
#include "GRSF/Dynamics/General/MultiBodySimFile.hpp"
#include "GRSF/Common/LogDefines.hpp"

class SimFileInfo{;
public:

    using XMLDocumentType = pugi::xml_document;
    using XMLNodeType =  pugi::xml_node;

    struct ResampleInfo{
        std::streamsize m_startIdx;
        std::streamoff  m_endIdx;
        std::streamsize m_increment;

        inline std::string getString(std::string linePrefix="\t"){
            std::stringstream ss;
            ss << linePrefix << "\t ResampleInfo: startIdx: " << m_startIdx << ", endIdx: "
            << m_endIdx << ", increment: " << m_increment << std::endl;
            return ss.str();
        }

        /** Appends the resample info to the XML node */
        void addXML(XMLNodeType & node){
            auto s = node.append_child("Resample");
            s.append_attribute("startIdx").set_value((long long int)m_startIdx);
            s.append_attribute("endIdx").set_value((long long int)m_endIdx);
            s.append_attribute("increment").set_value((long long int)m_increment);
        }

    };

    using DetailsList = std::vector< std::pair<MultiBodySimFile::Details,ResampleInfo> >;


    std::string getInfoString(const std::vector<boost::filesystem::path> & inputFiles,
                    std::streamsize increment = 1,
                    std::streamsize startStateIdx = 0,
                    std::streamsize endStateIdx = std::numeric_limits<unsigned int>::max(),
                    bool skipFirstState = true,
                    bool prettyPrint = false,
                    bool withTimeList = true)
    {
        // get  Description first then pretty print if needed
        DetailsList l;
        addInfo(l,inputFiles,increment,startStateIdx,endStateIdx,skipFirstState,withTimeList);

        if(!prettyPrint){
            auto doc = xmlDocument(l,withTimeList);
            std::stringstream ss;
            doc->save(ss);
            return ss.str();
        }else{
            return prettyString(l);
        }
    }

    std::string prettyString(DetailsList & d){
        std::stringstream s;
        for(auto & l : d){
            s << l.first.getString();
            s << l.second.getString();
        }
        return s.str();
    }

    std::unique_ptr<XMLDocumentType> xmlDocument(DetailsList & d, bool withTimeList){
        std::unique_ptr<XMLDocumentType>  doc(new XMLDocumentType{});
        XMLNodeType root = doc->append_child("SimInfo");
        for(auto & l : d){
            auto s = l.first.addXML(root,withTimeList);
            l.second.addXML(s);
        }
        return std::move(doc);
    }

    void addInfo(DetailsList & d,
                const std::vector<boost::filesystem::path> & inputFiles,
                std::streamsize increment = 1,
                std::streamsize startStateIdx = 0,
                std::streamsize endStateIdx = std::numeric_limits<unsigned int>::max(),
                bool skipFirstState = true,
                bool withTimeList = true) {

        // Track the startStateIdx for each file to resample
        // Skip all first states in all files except the first file,
        // we need to skip this because its the same state as the last files end state

        bool skip = skipFirstState;
        for(auto & i : inputFiles){
            if(skip){
                addInfoFile(d,i, increment, startStateIdx, endStateIdx, !skipFirstState, withTimeList );
                skip = false;
            }else{
                addInfoFile(d,i, increment, startStateIdx, endStateIdx, skipFirstState, withTimeList );
            }
        }
    }

private:

    void addInfoFile(DetailsList & detailList,
                        boost::filesystem::path f,
//                        std::streamsize & states,
                        const std::streamoff increment,
                        std::streamoff & startStateIdx,
                        std::streamoff & endStateIdx,
                        const bool skipFirstState,
                        bool withTimeList = true)
    {
        MultiBodySimFile fromFile;
        if(!fromFile.openRead(f)) {
            ERRORMSG(fromFile.getErrorString());
        };


        auto details = fromFile.getDetails(withTimeList);

        if (startStateIdx >= endStateIdx){
             detailList.emplace_back( std::make_pair(details,ResampleInfo{startStateIdx, startStateIdx,  0}) );
             return;
        }

        startStateIdx += skipFirstState? 1 : 0;
        endStateIdx   += skipFirstState? 1 : 0;

        std::streamoff statesFile = fromFile.getNStates();
//        states += statesFile - (skipFirstState? 1 : 0); // accumulate total states

        if(startStateIdx >= statesFile){
            // Resample Info: no resample
            startStateIdx -= statesFile; // skip this file subtract the number of states of this file
            endStateIdx   -= statesFile;
            detailList.emplace_back( std::make_pair(details,ResampleInfo{startStateIdx, startStateIdx,  0}) );
        }
        else{

            if ( endStateIdx < statesFile){
                detailList.emplace_back( std::make_pair(details, ResampleInfo{startStateIdx, endStateIdx,  increment}));
                endStateIdx = 0;

            }else{
                detailList.emplace_back( std::make_pair(details, ResampleInfo{startStateIdx, -1,  increment}) );
                endStateIdx -= statesFile;
            }

            // compute carry over for next state
            startStateIdx = (statesFile - startStateIdx) % increment ; // how many states
//            startStateIdx = (( n  + increment-1 ) / increment) * increment  - n;
            // (( n  + increment-1 ) / increment)  = ceil ( n / increment) = how many states we took
        }

    }



};

#endif // SimFileInfo_hpp



