// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include <iostream>
#include <stdio.h>

//#include "MpiTests.hpp"
#include "SerializationTests.hpp"
#include "SerializationVariant.hpp"

using namespace std;

int main(int argc, char** argv)
{
    // mpi_greetings(argc,argv);
    // TypeTest();
    // TypeTestMPI(argc,argv);
    // fileTestMPI(argc,argv);

    // testSerialization();
    // testSerializeVariant();
    // testSerializePointer();
    // testSerializePointerClass();
    // testSerializationRigidBodyFile();
    // testSerializationRigidBodyString();
    // testSerializationRigidBodyMPI(argc,argv);
    // testSerializationRigidBodyMessage();
    // testSerializationRigidBodyMessageMPI(argc,argv);
    // testSerializationRigidBodyMessageBoostMPI(argc,argv);
    // testSerializationRigidBodyMessageBoostMPI2(argc,argv);
    // serializationTestClass();
    // testSerializationSpecialMessageMPI(argc,argv);

    testWaitAll(argc, argv);
}
