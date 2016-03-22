
#include<stdio.h>
#include<iostream>

//#include "MpiTests.hpp"
#include "SerializationTests.hpp"
#include "SerializationVariant.hpp"

using namespace std;

int main(int argc, char** argv){


    //mpi_greetings(argc,argv);
    //TypeTest();
    //TypeTestMPI(argc,argv);
    //fileTestMPI(argc,argv);

    //testSerialization();
    //testSerializeVariant();
    //testSerializePointer();
    //testSerializePointerClass();
    //testSerializationRigidBodyFile();
    //testSerializationRigidBodyString();
    //testSerializationRigidBodyMPI(argc,argv);
    //testSerializationRigidBodyMessage();
    //testSerializationRigidBodyMessageMPI(argc,argv);
    //testSerializationRigidBodyMessageBoostMPI(argc,argv);
    //testSerializationRigidBodyMessageBoostMPI2(argc,argv);
    //serializationTestClass();
    //testSerializationSpecialMessageMPI(argc,argv);

    testWaitAll(argc, argv);
}



