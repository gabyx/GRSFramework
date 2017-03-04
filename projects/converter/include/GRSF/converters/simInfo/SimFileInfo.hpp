// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_converters_simInfo_SimFileInfo_hpp
#define GRSF_converters_simInfo_SimFileInfo_hpp

#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <vector>

#include <boost/filesystem.hpp>
#include "pugixml.hpp"

#include "GRSF/common/LogDefines.hpp"
#include "GRSF/common/ProgressBarCL.hpp"
#include "GRSF/dynamics/general/MultiBodySimFile.hpp"

class SimFileInfo
{
    ;

public:
    using XMLDocumentType = pugi::xml_document;
    using XMLNodeType     = pugi::xml_node;

    struct ResampleInfo
    {
        using StateIndices = std::vector<std::streamsize>;

        ResampleInfo(std::streamsize startIdx,
                     std::streamoff endIdx,
                     std::streamsize increment,
                     std::streamsize nStates)
            : m_startIdx(startIdx), m_endIdx(endIdx), m_increment(increment), m_nStates(nStates)
        {
        }

        inline std::string getString(std::string linePrefix = "\t")
        {
            std::stringstream ss;
            ss << linePrefix << "\t ResampleInfo: startIdx: " << m_startIdx << ", endIdx: " << m_endIdx
               << ", increment: " << m_increment << std::endl;
            return ss.str();
        }

        inline StateIndices getIndices()
        {
            StateIndices res;
            if (m_increment != 0)
            {
                // this is a file where we take states (increment=0 no states to take)
                auto end = (m_endIdx == -1) ? m_nStates : m_endIdx;
                for (std::streamsize i = m_startIdx; i < end; i += m_increment)
                {
                    res.emplace_back(i);
                }
            }
            return res;
        }

        /** Appends the resample info to the XML node */
        void addXML(XMLNodeType& node, bool withIndices = true)
        {
            auto s = node.append_child("Resample");
            s.append_attribute("startIdx").set_value((long long int)m_startIdx);
            s.append_attribute("endIdx").set_value((long long int)m_endIdx);
            s.append_attribute("increment").set_value((long long int)m_increment);

            if (withIndices)
            {
                auto indices = getIndices();
                if (!indices.empty())
                {
                    std::stringstream ss;
                    ss << indices[0];
                    for (std::size_t i = 1; i < indices.size(); ++i)
                    {
                        ss << " " << indices[i];
                    }
                    // save to xml
                    static const auto nodePCData = pugi::node_pcdata;
                    XMLNodeType node             = s.append_child("Indices");
                    node.append_child(nodePCData).set_value(ss.str().c_str());
                }
            }
        }

    private:
        std::streamsize m_startIdx;
        std::streamoff m_endIdx;
        std::streamsize m_increment;
        std::streamsize m_nStates;
    };

    using DetailsList = std::vector<std::pair<MultiBodySimFile::Details, ResampleInfo>>;

    std::string getInfoString(const std::vector<boost::filesystem::path>& inputFiles,
                              std::streamsize increment     = 1,
                              std::streamsize startStateIdx = 0,
                              std::streamsize endStateIdx   = std::numeric_limits<unsigned int>::max(),
                              bool skipFirstState           = true,
                              bool prettyPrint              = false,
                              bool withTimeList             = true)
    {
        // get  Description first then pretty print if needed
        DetailsList l;
        addInfo(l, inputFiles, increment, startStateIdx, endStateIdx, skipFirstState, withTimeList);

        if (!prettyPrint)
        {
            auto doc = xmlDocument(l, withTimeList);
            std::stringstream ss;
            doc->save(ss);
            return ss.str();
        }
        else
        {
            return prettyString(l);
        }
    }

    std::string prettyString(DetailsList& d)
    {
        std::stringstream s;
        for (auto& l : d)
        {
            s << l.first.getString();
            s << l.second.getString();
        }
        return s.str();
    }

    std::unique_ptr<XMLDocumentType> xmlDocument(DetailsList& d, bool withTimeList)
    {
        std::unique_ptr<XMLDocumentType> doc(new XMLDocumentType{});
        XMLNodeType root = doc->append_child("SimInfo");
        for (auto& l : d)
        {
            auto s = l.first.addXML(root, withTimeList);
            l.second.addXML(s);
        }
        return std::move(doc);
    }

    void addInfo(DetailsList& d,
                 const std::vector<boost::filesystem::path>& inputFiles,
                 std::streamsize increment     = 1,
                 std::streamsize startStateIdx = 0,
                 std::streamsize endStateIdx   = std::numeric_limits<unsigned int>::max(),
                 bool skipFirstState           = true,
                 bool withTimeList             = true)
    {
        // Track the startStateIdx for each file to resample
        // Skip all first states in all files except the first file,
        // we need to skip this because its the same state as the last files end state

        bool skip = skipFirstState;
        for (auto& i : inputFiles)
        {
            if (skip)
            {
                addInfoFile(d, i, increment, startStateIdx, endStateIdx, !skipFirstState, withTimeList);
                skip = false;
            }
            else
            {
                addInfoFile(d, i, increment, startStateIdx, endStateIdx, skipFirstState, withTimeList);
            }
        }
    }

private:
    void addInfoFile(DetailsList& detailList,
                     boost::filesystem::path f,
                     //                        std::streamsize & states,
                     const std::streamoff increment,
                     std::streamoff& startStateIdx,
                     std::streamoff& endStateIdx,
                     const bool skipFirstState,
                     bool withTimeList = true)
    {
        MultiBodySimFile fromFile;
        if (!fromFile.openRead(f))
        {
            GRSF_ERRORMSG(fromFile.getErrorString());
        };

        auto details              = fromFile.getDetails(withTimeList);
        std::streamoff statesFile = fromFile.getNStates();

        if (startStateIdx >= endStateIdx)
        {
            detailList.emplace_back(std::make_pair(details, ResampleInfo{startStateIdx, startStateIdx, 0, statesFile}));
            return;
        }

        // shift range by one if we skip first state!
        startStateIdx += skipFirstState ? 1 : 0;
        endStateIdx += skipFirstState ? 1 : 0;

        //        states += statesFile - (skipFirstState? 1 : 0); // accumulate total states

        if (startStateIdx >= statesFile)
        {
            // Resample Info: no resample
            startStateIdx -= statesFile;  // skip this file subtract the number of states of this file
            endStateIdx -= statesFile;
            detailList.emplace_back(std::make_pair(details, ResampleInfo{startStateIdx, startStateIdx, 0, statesFile}));
        }
        else
        {
            if (endStateIdx < statesFile)
            {
                detailList.emplace_back(
                    std::make_pair(details, ResampleInfo{startStateIdx, endStateIdx, increment, statesFile}));
                endStateIdx = 0;
            }
            else
            {  // endStateIdx >= statesFile
                detailList.emplace_back(
                    std::make_pair(details, ResampleInfo{startStateIdx, -1, increment, statesFile}));
                endStateIdx -= statesFile;
            }

            // compute carry over for next state
            auto n        = (statesFile - startStateIdx);  // how many states
            startStateIdx = ((n + increment - 1) / increment) * increment - n;
            // (( n  + increment-1 ) / increment)  = ceil ( n / increment) = how many states we took
            // subtract this from n, to get carry over
        }
    }
};

#endif  // SimFileInfo_hpp
