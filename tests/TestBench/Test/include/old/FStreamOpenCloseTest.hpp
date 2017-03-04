// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef FStreamOpenCloseTest_hpp
#define FStreamOpenCloseTest_hpp

#include <chrono>
#include <iostream>
#include <stdio.h>

void fstreamOpenCloseTest()
{
    const unsigned int increment = 1024 * 8;

    unsigned int writeNTimes = 100;  // Cycle n times over all files and write some shit to them

    const unsigned int nFiles = 1000;

    std::string add = "100Files.txt";
    unsigned int loops, restBytes;

    std::ofstream output1("FStreamTest-AllOpenCloseTest-" + add);
    output1 << "#WriteLength \tTimeToWrite \tWriteSpeedPerFile [mb/s]" << std::endl;

    std::ofstream output2("FStreamTest-LeaveOpen-" + add);
    output2 << "#WriteLength \tTimeToWrite \tWriteSpeedPerFile [mb/s]" << std::endl;

    typedef std::chrono::duration<double> fsec;
    typedef std::chrono::high_resolution_clock Clock;

    // Test Data for the Buffer
    char value     = 1;
    char* testData = new char[increment];  // Just Garbage
    std::memset(testData, value, increment);

    // Make all files:
    std::vector<std::string> vecFiles;
    std::stringstream s;
    for (int i = 0; i < nFiles; i++)
    {
        s.str("");
        s << "TestFile_" << i << ".dat";
        vecFiles.push_back(s.str());
        std::fstream(s.str(), std::ios::binary | std::ios::trunc | std::ios::out);
    }

    unsigned int bufL = increment;

    std::cout << "Test open/close: " << bufL << std::endl;
    // Open close
    //        {
    //            auto t1 = Clock::now();
    //            for(int i =0; i < writeNTimes;i++){
    //
    //                std::fstream stream;
    //
    //                for(auto it =  vecFiles.begin(); it != vecFiles.end() ; it++){
    //                    stream.open(*it, std::ios::binary | std::ios::app | std::ios::out);
    //                    stream.write(testData,bufL);
    //                    stream.close();
    //                }
    //
    //            }
    //            auto t2 = Clock::now();
    //
    //            //Calculate timing
    //            fsec time = (t2 - t1);
    //            output1 << bufL << "\t" << time.count() <<"\t" << (bufL/(time.count()/(nFiles*writeNTimes))) /
    //            (1024*1024) << std::endl;
    //        }

    {
        // open all files
        std::vector<std::fstream*> vecStream;
        std::stringstream s;
        for (int i = 0; i < nFiles; i++)
        {
            s.str("");
            s << "TestFile_" << i << ".dat";
            vecStream.push_back(new std::fstream(s.str(), std::ios::binary | std::ios::app | std::ios::out));
        }

        auto t1 = Clock::now();
        for (int i = 0; i < writeNTimes; i++)
        {
            for (auto it = vecStream.begin(); it != vecStream.end(); it++)
            {
                (*it)->write(testData, bufL);
            }
        }
        auto t2 = Clock::now();

        for (auto it = vecStream.begin(); it != vecStream.end(); it++)
        {
            (*it)->close();
        }

        // Calculate timing
        fsec time = (t2 - t1);
        output2 << bufL << "\t" << time.count() << "\t"
                << (bufL / (time.count() / (nFiles * writeNTimes))) / (1024 * 1024) << std::endl;
    }

    for (int i = 0; i < nFiles; i++)
    {
        s.str("");
        s << "TestFile_" << i << ".dat";
        remove(s.str().c_str());
    }
}

#endif
