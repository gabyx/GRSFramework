#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>

#include <Eigen/Dense>

#include <mpi.h>

#include <fstream>

#include "RigidBody.hpp"

//#include <boost/mpi/environment.hpp>
//#include <boost/mpi/communicator.hpp>
//#include <boost/mpi/skeleton_and_content.hpp>

// include headers that implement a archive in simple text format
//
//#include <boost/array.hpp>
//#include <boost/iostreams/stream.hpp>
//#include <boost/iostreams/device/back_inserter.hpp>
//#include <boost/archive/text_oarchive.hpp>
//#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/split_member.hpp>
//#include <boost/serialization/vector.hpp>
//#include <boost/serialization/variant.hpp>
//#include <boost/serialization/shared_ptr.hpp>
//#include <boost/serialization/string.hpp>
#include <boost/serialization/version.hpp>


class IntPtrClass {
public:
    int * a;
    IntPtrClass(int i) {
        a = new int(i);
    }
    ~IntPtrClass() {
        delete a;
    }
};

namespace boost {
namespace serialization {

template<class Archive>
inline void save_construct_data(
    Archive & ar, const int * foo, const unsigned int file_version
){
    ar << *foo;
}

template<class Archive>
inline void load_construct_data(
    Archive & ar, int * foo, const unsigned int file_version
){
    if(!foo){
        std::cout << "load_construct_data:: making new int()" <<std::endl;
        foo =  new int();
    }
    ar >> *foo;
}

template<class Archive>
void serialize(Archive & ar, std::string* & g, const unsigned int version) {
    std::cout << "Serialize std::string*" << std::endl;
    if(!g) {
        g = new std::string();
    }
    //serialize(ar, *g);
}

template<class Archive>
void serialize(Archive & ar, boost::variant<int*, std::string *> & g, const unsigned int version) {


    if(Archive::is_loading::value){
        std::cout << "Deserialize variant..." << std::endl;
        int which; ar & which;

        if(which==0) {
            if(g.which()!=which){
                std::cout << "input is not int, make new!" << std::endl;
                g =  new int();
            }
            int * ptr = boost::get<int*>(g);
            ar & *ptr;
        }
        else if(which==1)
        {
            if(g.which()!=which){
                std::cout << "input is not string, make new!" << std::endl;
                g =  new std::string();
            }
            std::string * ptr = boost::get<std::string*>(g);
            ar & *ptr;
        }

    }else{
        std::cout << "Serialize variant..." << std::endl;
        int a = g.which();
        ar & a;
        if(g.which()==0) {
            int * ptr = boost::get<int*>(g);
            ar & *ptr;
        }
        else if(g.which()==1)
        {
            std::string * ptr = boost::get<std::string*>(g);
            ar & *ptr;
        }
    }
    //serialize(ar, *g);
}

template<class Archive>
void serialize(Archive & ar, int* & g, const unsigned int version) {
    std::cout << "Serialize int*" << std::endl;
    if(!g) {
        g = new int();
    }
    //ar & *g;
}

template<class Archive>
void serialize(Archive & ar, IntPtrClass & g, const unsigned int version) {
    std::cout << "Serialize IntPtrClass.a" << std::endl;
// This code serializes in place if the pointer exists
//    if(!g.a) {
//        g.a = new int();
//    }
//    ar & *(g.a);

    //ar & g.a;
}

}
}

void testSerializeVariant(){
    std::stringstream ss;
    boost::variant<int*,std::string* > var;

    var = new std::string("aaaaaaaaaaaaaaaa");
    std::cout<<"Variant to serialize which:"<< var.which() << " : std::string*: "<<
    boost::get<std::string*>(var)<< *(boost::get<std::string*>(var))<< std::endl;
    // save data
    std::cout<<"Serializing Variant: size"<< sizeof(var) << std::endl;
    {
        boost::archive::binary_oarchive oa(ss);
        oa << var;
    }


    boost::variant<int*,std::string* > var2;
    std::string * ptr = new std::string("bbb");
    //int * ptr = new int(2);
    var2 = ptr;
    std::cout<<"Variant before which:"<< var2.which() << ",  "<< ptr<< std::endl;

    {
        boost::archive::binary_iarchive is(ss);
        is >> var2;
    }

    if(var2.which()==0) {
        int * ptr = boost::get<int*>(var2);
        std::cout<<"Variant which: "<< var2.which() << ": int*:" <<ptr<< ", "<<*ptr <<std::endl;
    } else {
        std::string * ptr = boost::get<std::string*>(var2);
        std::cout<<"Variant which:"<< var2.which() << " : std::string*: "<< ptr<< ", "<<*ptr << std::endl;
    }
}

void testSerializePointer(){

std::stringstream ss;


    int * var = new int(1);

    // save data
    std::cout<<"Serializing Variant: size"<< sizeof(var) << std::endl;
    {
       boost::archive::binary_oarchive oa(ss);
       //oa << var;
    }


    int * var2 = new int(2);

    std::cout<<"Deserializing Variant: size"<< sizeof(var2) << " initial @: "<< var2<< std::endl;

    boost::archive::binary_iarchive is(ss);
    // read class state from archive
    //is >> var2;

    std::cout<<"Variant: int*  @: "<< var2<< std::endl;

}



void testSerializePointerClass(){
        std::stringstream ss;


        IntPtrClass var(3);

        // save data
        std::cout<<"Serializing Class IntPtrClass: size"<< sizeof(var) << std::endl;
        {
            boost::archive::binary_oarchive oa(ss);
            oa << var;
        }

        IntPtrClass var2(2);

        std::cout<<"Deserializing Class A: Initial Class IntPtrClass.a @: "<< var2.a<< "with: "<< *var2.a << std::endl;
        {
            boost::archive::binary_iarchive is(ss);;
            // read class state from archive
            is >> var2;
        }



        std::cout<<"Class IntPtrClass.a  @: "<< var2.a<< "with: "<< *var2.a <<std::endl;

}
