#ifndef SimFileInfo_hpp
#define SimFileInfo_hpp

#include <iostream>
#include <iomanip>
#include <set>
#include <map>
#include <vector>

#include <boost/filesystem.hpp>
#include "GRSF/Common/ProgressBarCL.hpp"
#include "GRSF/Dynamics/General/MultiBodySimFile.hpp"


#include "GRSF/Common/LogDefines.hpp"


class SimFileInfo{;
public:


    std::string getInfo(const std::vector<boost::filesystem::path> & inputFiles,
                        std::streamsize stepSize = 1,
                        std::streamsize startStateIdx = 0,
                        std::streamsize endStateIdx = std::numeric_limits<unsigned int>::max(),
                        bool skipFirstState = true) {

        std::stringstream s;

        // Track the startStateIdx for each file to resample
        // Skip all first states in all files except the first file,
        // we need to skip this because its the same state as the last files end state
        std::streamsize states = 0;

        bool skip = skipFirstState;
        for(auto & i : inputFiles){
            if(skip){
                s << getInfoFile(i, states, stepSize, startStateIdx, endStateIdx, !skipFirstState );
                skip = false;
            }else{
                s << getInfoFile(i, states, stepSize, startStateIdx, endStateIdx, skipFirstState );
            }

        }

        return s.str();
    }

private:

    std::string getInfoFile(boost::filesystem::path f,
                            std::streamsize & states,
                            const std::streamsize stepSize,
                            std::streamsize & startStateIdx,
                            const std::streamsize endStateIdx,
                            const bool skipFirstState){

        std::stringstream s;

        MultiBodySimFile fromFile;

        if(!fromFile.openRead(f)) {
            ERRORMSG(fromFile.getErrorString());
        };

        s << fromFile.getDetails(true,"\t") << std::endl;

        startStateIdx += skipFirstState? 1 : 0;

        std::streamoff statesFile = fromFile.getNStates();
        states += statesFile - skipFirstState? 1 : 0;

        if(startStateIdx >= statesFile){
            s << "\t Resample Info: no resample" << std::endl;
            startStateIdx -= statesFile; // skip this file subtract the number of states of this file
        }else{



            s <<" \t Resample Info: " << "startIdx:" << startStateIdx ;
            if ( endStateIdx >= states){
                  s << " endIdx: " << "--";
            }else{
                 s << " endIdx: " <<  endStateIdx;
            }
            s << " stepSize: " << stepSize << std::endl;

            // ((statesFile + startStateIdx -1 ) / stepSize)  how many stepSize blocks we need at most
            auto n = (statesFile - startStateIdx) ;
            std::cout << "n:" << (( n  + stepSize-1 ) / stepSize) << std::endl;
            startStateIdx = (( n  + stepSize-1 ) / stepSize) * stepSize  - n;
        }


        return s.str();
    }

};

#endif // SimFileInfo_hpp



