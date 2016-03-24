// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef FStreamBufferTest_hpp
#define FStreamBufferTest_hpp

#include <stdio.h>
#include <cstring>
#include <iostream>
#include <fstream>
#include <chrono>


#define TOCOUT(output) \
    if(!outputToCout) { \
        buf = output##_t.rdbuf(); \
    } else { \
        buf = std::cout.rdbuf(); \
    } \
    std::ostream output(buf);

void fstreamBufferTest(){

    const bool outputToCout = false;

    const unsigned int multiplyStep = 1<<2;
    const unsigned int startLength = 1<<2;
    const unsigned int stopLength = 1<<24;

    const unsigned int writeNTimes = 1; // Averaging over some many times!
    const unsigned int fileLength = 1<< 30; //104857600=100mb,  314572800=300mb , 1<< 30 =1GB
    std::string add = "1000.txt";
    unsigned int loops, restBytes;


    std::streambuf * buf;

    std::ofstream output1_t("FStreamTest-FstreamBuffering-OwnBufferSet-"+add);
    TOCOUT(output1);
    output1 << "#Buffer Length \tTimeToWrite \tWriteSpeed [mb/s]" << std::endl;

    std::ofstream output2_t("FStreamTest-ManualBuffering-StdStreamBuffer-"+add);
    TOCOUT(output2);
    output2 << "#Buffer Length \tTimeToWrite \tWriteSpeed [mb/s]" << std::endl;

    std::ofstream output3_t("FStreamTest-ManualBuffering-NoInternalStreamBuffer-"+add);
    TOCOUT(output3);
    output3 << "#Buffer Length \tTimeToWrite \tWriteSpeed [mb/s]" << std::endl;

    std::ofstream output4_t("FStreamTest-NoManualBuffering-NoInternalStreamBuffer-"+add);
    TOCOUT(output4);
    output4 << "#Buffer Length \tTimeToWrite\tWriteSpeed [mb/s]" << std::endl;

    std::ofstream output5_t("FStreamTest-NoManualBuffering-StdStreamBuffer-"+add);
    TOCOUT(output5);
    output5 << "#Buffer Length \tTimeToWrite \tWriteSpeed [mb/s]" << std::endl;

    // To Cout


    typedef std::chrono::duration<double> fsec;
    typedef std::chrono::high_resolution_clock Clock;



    // Test Data for the Buffer
    bool removeFile = true;
    char value = 1;
    char *testData = new char[fileLength]; // Just Garbage 1GB!!
    std::memset(testData,value,fileLength);

    // Preallocate file;
    if(!removeFile){
        std::fstream stream;
        stream.open("test.dat", std::ios::binary | std::ios::trunc | std::ios::out);
        for(int i = 0; i < writeNTimes; i++){
                stream.write(testData, fileLength );
        }
        stream.close();
    }else{
        if( remove( "test.dat" ) == 0){
            std::cout << "File deleted at start!" << std::endl;
        }
    }

    for(unsigned int bufL = startLength; bufL <= stopLength; bufL = bufL * multiplyStep){

        // First Test with Fstream Buffering!
        {
            std::cout << "Doing test: FStream Buffering: " << bufL <<std::endl;
            char * buffer = new char[bufL];
            //open Stream
            std::fstream stream;
            stream.rdbuf()->pubsetbuf(buffer, bufL);
            stream.open("test.dat", std::ios::binary | std::ios::trunc | std::ios::out);

            // Write whole 1gb file! we have fstream buffering the stuff
            auto t1 = Clock::now();
            for(int i = 0; i < writeNTimes; i++){
                stream.write(testData, fileLength );
            }
            stream.close();
            auto t2 = Clock::now();

            //Calculate timing
            fsec time = (t2 - t1) / writeNTimes;
            output1 << bufL << "\t" << time.count() <<"\t" << (fileLength/time.count()) / (1024*1024) << std::endl;

            delete buffer;
            if(removeFile){
                if( remove( "test.dat" ) != 0){
                    std::cerr << "File not deleted" << std::endl;
                };
            }
        }

        // Second Test with Manual Buffering!
        {
            std::cout << "Doing test: Manual Buffering: " << bufL <<std::endl;
            // Calculate the loops to write fileLength
            loops = fileLength / bufL;
            restBytes =  fileLength % bufL;

            //open Stream
            std::fstream stream;
            stream.open("test.dat", std::ios::binary | std::ios::trunc | std::ios::out);
            // TODO stream buf -> 0

            // Write 1GB File in loops of bufL
            auto t1 = Clock::now();
            for(int i = 0; i < writeNTimes;  i++){
                for(int i = 0; i < loops; i++){
                   stream.write(testData, bufL );
                }
                stream.write(testData, restBytes );
            }
            stream.close();
            auto t2 = Clock::now();

            //Calculate timing
            fsec time = (t2 - t1) / writeNTimes;
            output2 << bufL << "\t" << time.count() <<"\t" << (fileLength/time.count()) / (1024*1024) << std::endl;
            if(removeFile){
                if( remove( "test.dat" ) != 0){
                    std::cerr << "File not deleted" << std::endl;
                };
            }
        }

        // Second Test with Manual Buffering!
        {
            std::cout << "Doing test: Manual Buffering (no internal stream buffer): " << bufL <<std::endl;
            // Calculate the loops to write fileLength
            loops = fileLength / bufL;
            restBytes =  fileLength % bufL;

            //open Stream
            std::fstream stream;
            stream.open("test.dat", std::ios::binary | std::ios::trunc | std::ios::out);
            stream.rdbuf()->pubsetbuf(0, 0);

            // Write 1GB File in loops of bufL
            auto t1 = Clock::now();
            for(int i = 0; i < writeNTimes;  i++){
                for(int i = 0; i < loops; i++){
                   stream.write(testData, bufL );
                }
                stream.write(testData, restBytes );
            }
            stream.close();
            auto t2 = Clock::now();

            //Calculate timing
            fsec time = (t2 - t1) / writeNTimes;
            output3 << bufL << "\t" << time.count() <<"\t" << (fileLength/time.count()) / (1024*1024) << std::endl;
            if(removeFile){
                if( remove( "test.dat" ) != 0){
                    std::cerr << "File not deleted" << std::endl;
                };
            }
        }


        {
            std::cout << "Doing test: No manual Buffering (no internal stream buffer): " << bufL <<std::endl;
            // Calculate the loops to write fileLength
            loops = fileLength / bufL;
            restBytes =  fileLength % bufL;

            //open Stream
            std::fstream stream;
            stream.open("test.dat", std::ios::binary | std::ios::trunc | std::ios::out);
            stream.rdbuf()->pubsetbuf(0, 0);

            // Write 1GB File in loops of bufL
            auto t1 = Clock::now();
            for(int i = 0; i < writeNTimes;  i++){
                stream.write(testData, fileLength );
            }
            stream.close();
            auto t2 = Clock::now();

            //Calculate timing
            fsec time = (t2 - t1) / writeNTimes;
            output4 << bufL << "\t" << time.count() <<"\t" << (fileLength/time.count()) / (1024*1024) << std::endl;
            if(removeFile){
                if( remove( "test.dat" ) != 0){
                    std::cerr << "File not deleted" << std::endl;
                };
            }
        }

        {
            std::cout << "Doing test: No manual Buffering (std stream buffer): " << bufL <<std::endl;
            //Calculate the loops to write fileLength
            loops = fileLength / bufL;
            restBytes =  fileLength % bufL;

            //open Stream
            std::fstream stream;
            stream.open("test.dat", std::ios::binary | std::ios::trunc | std::ios::out);

            // Write 1GB File in loops of bufL
            auto t1 = Clock::now();
            for(int i = 0; i < writeNTimes;  i++){
                stream.write(testData, fileLength );
            }
            stream.close();
            auto t2 = Clock::now();

            //Calculate timing
            fsec time = (t2 - t1)/ writeNTimes;
            output5 << bufL << "\t" << time.count() <<"\t" << (fileLength/time.count()) / (1024*1024) << std::endl;
            if(removeFile){
                if( remove( "test.dat" ) != 0){
                    std::cerr << "File not deleted" << std::endl;
                };
            }
        }



    }



}


#endif
