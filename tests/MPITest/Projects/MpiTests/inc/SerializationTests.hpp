// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include <Eigen/Dense>

#include <mpi.h>

#include <fstream>

#include "RigidBody.hpp"

#include <boost/mpi/communicator.hpp>
#include <boost/mpi/environment.hpp>
#include <boost/mpi/skeleton_and_content.hpp>

// include headers that implement a archive in simple text format

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/array.hpp>
#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/iostreams/stream.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/variant.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/version.hpp>
/////////////////////////////////////////////////////////////
// gps coordinate
//
// illustrates serialization for a simple type
//

class gps_position
{
    private:
    friend class boost::serialization::access;
    // When the class Archive corresponds to an output archive, the
    // & operator is defined similar to <<.  Likewise, when the class Archive
    // is a type of input archive the & operator is defined similar to >>.
    template <class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar& degrees;
        ar& minutes;
        ar& seconds;
    }
    int   degrees;
    int   minutes;
    float seconds;

    public:
    gps_position(){};
    gps_position(int d, int m, float s) : degrees(d), minutes(m), seconds(s)
    {
    }
};

class Obj
{
    public:
    Obj(int b)
    {
        m_a = b;
    }
    Obj()
    {
        m_a = -1;
    }

    int m_a;

    template <class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar& m_a;
    }

    ~Obj()
    {
        std::cout << "Destructing Obj: @" << this << std::endl;
    }
};

class serialTestClass
{
    public:
    BOOST_SERIALIZATION_SPLIT_MEMBER();

    serialTestClass(int a)
    {
        ptr1 = new Obj(a);
    }

    template <class Archive>
    void save(Archive& ar, const unsigned int version) const
    {
        // ar & *ptr1;

        // ar & ptr1; //Does not work
    }

    template <class Archive>
    void load(Archive& ar, const unsigned int version)
    {
        // ar & ptr1; // Does not work, does not delete pointer

        if (ptr1 != 0)
        {
            ar&* ptr1;
        }
        else
        {
            ptr1 = new Obj(-1);
            ar&* ptr1;
        }
    }

    void print(std::ostream& f = std::cout)
    {
        f << "ptr1 @" << ptr1 << std::endl;
        f << "ptr1->m_a :" << ptr1->m_a << std::endl;
    }

    Obj* ptr1;
};

void serializationTestClass()
{
    std::ofstream ofs("filename");

    serialTestClass b(1);

    std::cout << "Marshalling TestClass: size" << sizeof(b) << std::endl;
    b.print();

    // save data
    {
        boost::archive::binary_oarchive oa(ofs);
        oa << b;
    }

    // ... some time later restore the class instance to its orginal state
    std::cout << "Unmarshalling Body:" << std::endl;
    serialTestClass b2(2);  // INITIALIZE (POINTER IS NOT ZERO!)
    b2.print();
    {
        std::ifstream                   ifs("filename");
        boost::archive::binary_iarchive ia(ifs);
        ia >> b2;
    }

    b2.print();
}

BOOST_CLASS_VERSION(MyRigidBody, 1)

void printBody(MyRigidBody& b, std::ostream& f = std::cout)
{
    f << "RigidBody: @" << &b << "====================================" << std::endl;
    if (boost::shared_ptr<BoxGeometry<MyRigidBody::PREC>>* ptr =
            boost::get<boost::shared_ptr<BoxGeometry<MyRigidBody::PREC>>>(&b.m_geometry))
    {
        f << "m_geometry: \t" << typeid(*ptr).name() << " @" << (*ptr).get() << std::endl;
        f << "\t\t extent:" << (*ptr)->m_extent << std::endl;
        f << "\t\t center:" << (*ptr)->m_center << std::endl;
    }

    f << "m_eMaterial: \t" << b.m_eMaterial << std::endl;
    f << "m_eState: \t" << b.m_eState << std::endl;
    f << "m_A_IK: \t" << std::endl << b.m_A_IK << std::endl;
    f << "m_q_KI: \t" << std::endl << b.m_q_KI.transpose() << std::endl;
    f << "m_mass: \t" << std::endl << b.m_mass << std::endl;
    f << "m_h_term: \t" << std::endl << b.m_h_term.transpose() << std::endl;
    f << "m_h_term_const: \t" << std::endl << b.m_h_term_const << std::endl;
    f << "m_MassMatrix_diag: \t" << std::endl << b.m_MassMatrix_diag << std::endl;
    f << "m_MassMatrixInv_diag: \t" << std::endl << b.m_MassMatrixInv_diag << std::endl;
    f << "m_K_Theta_S: \t" << std::endl << b.m_K_Theta_S.transpose() << std::endl;

    if (b.m_pSolverData)
        f << "m_pSolverData: \t"
          << "@" << b.m_pSolverData << std::endl;
    if (b.m_pSolverData)
    {
        f << "m_pSolverData->m_uBuffer.m_Front: \t" << b.m_pSolverData->m_uBuffer.m_Front.transpose() << std::endl;
        f << "m_pSolverData->m_uBuffer.m_Back: \t" << b.m_pSolverData->m_uBuffer.m_Back.transpose() << std::endl;
    }
    f << "RigidBody: END ====================================" << std::endl;
}

void fillBodyRandom(MyRigidBody& b)
{
    MyRigidBody::Vector3 temp1;
    temp1.setRandom();
    MyRigidBody::Vector3 temp2;
    temp2.setRandom();
    b.m_geometry = boost::shared_ptr<BoxGeometry<MyRigidBody::PREC>>(new BoxGeometry<MyRigidBody::PREC>(temp1, temp2));

    b.m_eMaterial = 0;
    b.m_eState    = MyRigidBody::ANIMATED;
    b.m_mass      = 0.3131123;
    b.m_K_Theta_S.setRandom();
    b.m_MassMatrix_diag.setRandom();
    b.m_MassMatrixInv_diag.setRandom();
    b.m_h_term.setRandom();
    b.m_h_term_const.setRandom();

    b.m_A_IK.setRandom();
    b.m_r_S.setRandom();
    b.m_q_KI.setRandom();

    if (!b.m_pSolverData)
    {
        b.m_pSolverData = new MyRigidBody::RigidBodySolverDataType();
    }

    b.m_pSolverData->m_uBuffer.m_Front.setRandom();
    b.m_pSolverData->m_uBuffer.m_Back.setRandom();
}

struct RigidBodyMessage
{
    std::vector<MyRigidBody*> bodies;

    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        // write own se/deserialize function for the values!
        int nBodies = bodies.size();  // If serialize, actual value!
        ar& nBodies;

        if (nBodies > bodies.size())
        {  // if serialize always false
            // adjust space
            for (int i = 0; i < (nBodies - bodies.size()); i++)
            {
                bodies.push_back(new MyRigidBody());
            }
            nBodies = bodies.size();
        }

        for (int i = 0; i < nBodies; i++)
        {
            ar&* bodies[i];
        }
    }

    RigidBodyMessage(){};

    RigidBodyMessage(int count)
    {
        for (int i = 0; i < count; i++)
        {
            bodies.push_back(new MyRigidBody());
        }
    }

    ~RigidBodyMessage()
    {
        for (int i = 0; i < bodies.size(); i++)
        {
            delete bodies[i];
        }
    }

    void fillRandom()
    {
        for (int i = 0; i < bodies.size(); i++)
        {
            fillBodyRandom(*bodies[i]);
        }
    }

    void print(std::ostream& f = std::cout)
    {
        f << "Message BEGIN, contains: " << bodies.size() << " Bodies ==============================" << std::endl;

        for (int i = 0; i < bodies.size(); i++)
        {
            f << " Body: " << i << "======= @" << bodies[i] << std::endl;
            printBody(*bodies[i], f);
            f << "=======================" << std::endl;
        }
        f << "Message END: " << bodies.size() << "==================================" << std::endl;
    }

    void printAddress(std::ostream& f = std::cout)
    {
        f << "Message BEGIN, contains: " << bodies.size() << " Bodies ==============================" << std::endl;
        for (int i = 0; i < bodies.size(); i++)
        {
            f << " Body: " << i << "======= @" << bodies[i] << std::endl;
        }
        f << "Message END: " << bodies.size() << "==================================" << std::endl;
    }
};

namespace boost
{
namespace serialization
{
template <class Archive, typename Derived>
void serializeEigen(Archive& ar, Eigen::EigenBase<Derived>& g, const unsigned int version)
{
    //            std::cout << "Serialize Eigen Object:"<<std::endl;
    //            std::cout << "   Size: " << g.size()<<std::endl;
    //            for(int i=0;i<g.size();i++){
    //                ar & *(g.derived().data() + i);
    //            }
    ar& boost::serialization::make_array(g.derived().data(), g.size());
}

template <class Archive, typename Derived>
void serialize(Archive& ar, Eigen::EigenBase<Derived>& g, const unsigned int version)
{
    //            std::cout << "Serialize Eigen Object:"<<std::endl;
    //            std::cout << "   Size: " << g.size()<<std::endl;
    //            for(int i=0;i<g.size();i++){
    //                ar & *(g.derived().data() + i);
    //            }
    ar& boost::serialization::make_array(g.derived().data(), g.size());
}

template <class Archive, typename TLayoutConfig>
void serialize(Archive& ar, RigidBodySolverDataCONoG<TLayoutConfig>& g, const unsigned int version)
{
    serializeEigen(ar, g.m_uBuffer.m_Back, version);
    serializeEigen(ar, g.m_uBuffer.m_Front, version);
    ar& g.m_bInContactGraph;
}

template <class Archive>
void serializeGeom(Archive& ar, typename MyRigidBody::GeometryType& g, const unsigned int version)
{
    //
    //                if(Archive::is_loading::value){
    //                    std::cout << " Deserialize geometry..."<<std::endl;
    //                    int which;
    //                    ar & which;
    //                    if(which == 2){
    //                       if(g.which() != which){
    //                           std::cout << "make new box!" << std::endl;
    //                            g = boost::shared_ptr<BoxGeometry< MyRigidBody::PREC> >(new BoxGeometry<
    //                            MyRigidBody::PREC>());
    //                       }
    //
    //                       boost::shared_ptr<BoxGeometry< MyRigidBody::PREC> >  ptr =
    //                       boost::get< boost::shared_ptr<BoxGeometry< MyRigidBody::PREC> > >(g);
    //                       serializeBox(ar,ptr,version);
    //
    //                    }
    //                }
    //                else{
    //                    std::cout << " Serialize geometry..." <<std::endl;
    //                    int a = g.which();
    //                    ar & a;
    //                    if(g.which()== 2){
    //                        boost::shared_ptr<BoxGeometry< MyRigidBody::PREC> > ptr =
    //                        boost::get< boost::shared_ptr<BoxGeometry< MyRigidBody::PREC> > >(g);
    //                        serializeBox(ar,ptr,version);
    //                    }
    //                }
    ar& g;
}

template <class Archive, typename PREC>
void serializeBox(Archive& ar, boost::shared_ptr<BoxGeometry<PREC>>& g, const unsigned int version)
{
    serializeEigen(ar, g->m_extent, version);
    serializeEigen(ar, g->m_center, version);
}

template <class Archive, typename PREC>
void serialize(Archive& ar, boost::shared_ptr<HalfspaceGeometry<PREC>>& g, const unsigned int version)
{
    serializeEigen(ar, g->m_normal, version);
    serializeEigen(ar, g->m_pos, version);
}

template <class Archive, typename PREC>
void serialize(Archive& ar, boost::shared_ptr<SphereGeometry<PREC>>& g, const unsigned int version)
{
    ar & g->m_radius;
}

template <class Archive, typename PREC>
void serialize(Archive& ar, BoxGeometry<PREC>& g, const unsigned int version)
{
    serializeEigen(ar, g.m_extent, version);
    serializeEigen(ar, g.m_center, version);
}

template <class Archive, typename PREC>
void serialize(Archive& ar, HalfspaceGeometry<PREC>& g, const unsigned int version)
{
    serializeEigen(ar, g.m_normal, version);
    serializeEigen(ar, g.m_pos, version);
}

template <class Archive, typename PREC>
void serialize(Archive& ar, SphereGeometry<PREC>& g, const unsigned int version)
{
    ar& g.m_radius;
}

template <class Archive, typename TRigidBodyConfig>
void serialize(Archive& ar, RigidBodyBase<TRigidBodyConfig>& g, const unsigned int version)
{
    if (Archive::is_loading::value)
    {
        bool hadData = false;
        ar&  hadData;
        if (hadData)
        {
            if (!g.m_pSolverData)
            {
                g.m_pSolverData = new typename RigidBodyBase<TRigidBodyConfig>::RigidBodySolverDataType();
            }
            ar&* g.m_pSolverData;
        }
    }
    else
    {
        bool b = false;
        if (g.m_pSolverData)
        {
            b = true;
            ar&(b);
            ar&* g.m_pSolverData;
        }
        else
        {
            b = false;
            ar& b;
        }
    }

    if (Archive::is_loading::value)
    {
        ar& g.m_globalGeomId;
        if (g.m_globalGeomId == 0)
        {
            // ar & g.m_geometry;
            serializeGeom(ar, g.m_geometry, version);
        }
    }
    else
    {
        ar& g.m_globalGeomId;
        if (g.m_globalGeomId == 0)
        {
            // ar & g.m_geometry;
            serializeGeom(ar, g.m_geometry, version);
        }
    }

    // ar & g.m_geometry;

    ar& g.m_eMaterial;
    ar& g.m_eState;
    //
    ar& g.m_mass;
    serializeEigen(ar, g.m_K_Theta_S, version);
    serializeEigen(ar, g.m_MassMatrix_diag, version);
    serializeEigen(ar, g.m_MassMatrixInv_diag, version);
    serializeEigen(ar, g.m_h_term, version);
    serializeEigen(ar, g.m_h_term_const, version);
    ////
    serializeEigen(ar, g.m_A_IK, version);
    ;
    serializeEigen(ar, g.m_r_S, version);
    serializeEigen(ar, g.m_q_KI, version);
}

};  // namespace serialization
};  // namespace boost

int testSerializationRigidBodyFile()
{
    // create and open a character archive for output
    std::ofstream ofs("filename");

    // create class instancemy
    MyRigidBody b;
    fillBodyRandom(b);

    std::cout << "Marshalling Body: size" << sizeof(b) << std::endl;
    printBody(b);

    // save data to archive
    {
        boost::archive::binary_oarchive oa(ofs);
        oa << b;
    }
    std::cout << "STUB: Sending Data (serializable string) over Network!!..." << std::endl;
    // ... some time later restore the class instance to its orginal state
    MyRigidBody b2;
    fillBodyRandom(b2);
    std::cout << "Unmarshalling Body:" << std::endl;
    {
        // create and open an archive for input
        std::ifstream                   ifs("filename");
        boost::archive::binary_iarchive ia(ifs);

        // read class state from archive
        ia >> b2;
        // archive and stream closed when destructors are called
    }

    printBody(b2);
}

int testSerializationRigidBodyString()
{
    std::string serial_str;

    // create class instancemy
    MyRigidBody b;
    fillBodyRandom(b);
    std::cout << "Marshalling Body: size" << sizeof(b) << std::endl;
    printBody(b);

    // save data to archive
    {
        serial_str.clear();  // Clear serializable string!
        boost::iostreams::back_insert_device<std::string>                           inserter(serial_str);
        boost::iostreams::stream<boost::iostreams::back_insert_device<std::string>> s(inserter);
        boost::archive::binary_oarchive                                             oa(s);

        oa << b;
    }

    std::cout << "STUB: Sending Data (serializable string) over Network!!..." << std::endl;

    // ... some time later restore the class instance to its orginal state
    MyRigidBody b2;
    fillBodyRandom(b2);
    std::cout << "Unmarshalling Body:" << std::endl;
    {
        boost::iostreams::basic_array_source<char> device(serial_str.data(), serial_str.size());
        boost::iostreams::stream<boost::iostreams::basic_array_source<char>> s(device);
        boost::archive::binary_iarchive                                      ia(s);
        // read class state from archive
        ia >> b2;
        // archive and stream closed when destructors are called
    }

    printBody(b2);
}

int testSerializationRigidBodyMessage()
{
    // YOU CAN EITHER TAKE A Serial_str of Char_vector
    std::string       serial_str;
    std::vector<char> char_vector;

    // create class message
    RigidBodyMessage rbmess(3);
    rbmess.fillRandom();

    std::cout << " Message contains: " << rbmess.bodies.size() << "bodies" << std::endl;
    rbmess.print();

    std::cout << "Marshalling BodyMessage:" << std::endl;

    // save data to archive
    {
        char_vector.clear();
        serial_str.clear();  // Clear serializable string!
        boost::iostreams::back_insert_device<std::vector<char>>                           inserter(char_vector);
        boost::iostreams::stream<boost::iostreams::back_insert_device<std::vector<char>>> s(inserter);
        boost::archive::binary_oarchive                                                   oa(s);

        oa << rbmess;
    }

    std::cout << "STUB: Sending Data (serializable string) over Network!!..." << std::endl;

    // ... some time later restore the class instance to its orginal state
    RigidBodyMessage rbmess2(4);
    std::cout << "Unmarshalling BodyMessage:" << std::endl;
    {
        boost::iostreams::basic_array_source<char> device(&char_vector[0], char_vector.size() * sizeof(char));
        boost::iostreams::stream<boost::iostreams::basic_array_source<char>> s(device);
        boost::archive::binary_iarchive                                      ia(s);
        // read class state from archive
        ia >> rbmess2;
        // archive and stream closed when destructors are called
    }

    std::cout << " Message contains: " << rbmess2.bodies.size() << "bodies" << std::endl;
    rbmess2.print();
}

void sendBodyMPI(std::ostream& f = std::cout)
{
    f << "Sending Body (MPI)" << std::endl;

    int my_rank;
    my_rank = MPI::COMM_WORLD.Get_rank();

    usleep(100 * my_rank);

    if (my_rank > 3)
    {
        //        //Send Shit!
        //        int tag = 0;
        //        int dest = 0;
        //        MPI::Request request;
        //        char a = 3;
        //        MPI::Isend(&a, 1, MPI::CHAR, dest, 0, MPI::COMM_WORLD,&request);
    }
    else
    {
        // Send normal Body

        std::string serial_str;
        // create class instance

        MyRigidBody b;
        fillBodyRandom(b);
        f << "Marshalling Body: size" << sizeof(b) << std::endl;
        printBody(b, f);

        // save data to archive
        {
            serial_str.clear();  // Clear serializable string!
            boost::iostreams::back_insert_device<std::string>                           inserter(serial_str);
            boost::iostreams::stream<boost::iostreams::back_insert_device<std::string>> s(inserter);
            boost::archive::binary_oarchive                                             oa(s);
            oa << b;
        }

        // MPI SENDING to Rank 0!
        int          dest = 0;
        int          tag  = 0;
        MPI::Request request;
        request = MPI::COMM_WORLD.Isend(const_cast<char*>(serial_str.data()), serial_str.size(), MPI::CHAR, dest, tag);
    }
}

void receiveBodyMPI(int source, std::ostream& f = std::cout)
{
    f << "=========== Receving Body (MPI), ProcID," << source << " ==============" << std::endl;

    char*       buf;
    int         msglen = 0;
    int         tag    = 0;
    MPI::Status status;
    f << "Probe: " << std::endl;
    bool flag = false;

    // Polling
    while (1)
    {
        flag = MPI::COMM_WORLD.Iprobe(source, tag, status);  // Blocks
        f << "Waiting for Process Source:  " << source << std::endl;
        if (flag)
            break;
    }

    f << "Get Count: " << std::endl;
    msglen = status.Get_count(MPI::CHAR);
    f << " Should receive: " << msglen << std::endl;
    f << "Receive " << std::endl;
    buf = (char*)malloc(msglen * sizeof(char));
    MPI::COMM_WORLD.Recv(buf, msglen, MPI::CHAR, status.Get_source(), status.Get_tag());

    MyRigidBody b2;
    f << "Unmarshalling Body:" << std::endl;
    {
        boost::iostreams::basic_array_source<char>                           device(buf, msglen);
        boost::iostreams::stream<boost::iostreams::basic_array_source<char>> s(device);
        boost::archive::binary_iarchive                                      ia(s);
        // read class state from archive
        ia >> b2;
        // archive and stream closed when destructors are called
    }

    printBody(b2, f);

    free(buf);

    f << "=========================================================================" << std::endl;
}

int testSerializationRigidBodyMPI(int argc, char** argv)
{
    int         my_rank;
    int         p;
    int         source;
    int         dest;
    int         tag = 0;
    MPI::Status status;

    MPI::Init(argc, argv);

    my_rank = MPI::COMM_WORLD.Get_rank();
    p       = MPI::COMM_WORLD.Get_size();

    std::srand(my_rank);

    // Make file
    std::stringstream name;
    name << "ProcessLogID_" << my_rank;
    std::ofstream f(name.str());

    if (my_rank != 0)
    {
        sendBodyMPI(f);
    }
    else
    { /* my_rank = 0 */
        for (source = 1; source < p; source++)
        {
            receiveBodyMPI(source, f);
        }
    }

    MPI::Finalize();
}

void sendBodyMessageMPI(std::ostream& f = std::cout)
{
    f << "Sending BodyMessage (MPI)" << std::endl;

    int my_rank;
    my_rank = MPI::COMM_WORLD.Get_rank();

    usleep(100 * my_rank);

    if (my_rank > 3)
    {
        //        //Send Shit!
        //        int tag = 0;
        //        int dest = 0;
        //        MPI::Request request;
        //        char a = 3;
        //        MPI::Isend(&a, 1, MPI::CHAR, dest, 0, MPI::COMM_WORLD,&request);
    }
    else
    {
        // Send normal Body

        std::string serial_str;
        // create class instance

        RigidBodyMessage b(my_rank);
        b.fillRandom();

        b.print(f);

        f << "Marshalling Body: " << std::endl;

        // save data to archive
        {
            serial_str.clear();  // Clear serializable string!
            boost::iostreams::back_insert_device<std::string>                           inserter(serial_str);
            boost::iostreams::stream<boost::iostreams::back_insert_device<std::string>> s(inserter);
            boost::archive::binary_oarchive                                             oa(s);
            oa << b;
        }

        // MPI SENDING to Rank 0!
        int          dest = 0;
        int          tag  = 0;
        MPI::Request request;
        // f << "Sending String: " << serial_str <<std::endl;
        request = MPI::COMM_WORLD.Isend(const_cast<char*>(serial_str.data()), serial_str.size(), MPI::BYTE, dest, tag);
    }
}
void receiveBodyMessageMPI(int source, std::ostream& f = std::cout)
{
    f << "=========== Receving Body (MPI), ProcID," << source << " ==============" << std::endl;

    char*       buf;
    int         msglen = 0;
    int         tag    = 0;
    MPI::Status status;
    f << "Probe: " << std::endl;
    bool flag = false;

    // Polling
    while (1)
    {
        flag = MPI::COMM_WORLD.Iprobe(source, tag, status);  // Blocks
        f << "Waiting for Process Source:  " << source << std::endl;
        if (flag)
            break;
    }

    f << "Get Count: " << std::endl;
    msglen = status.Get_count(MPI::BYTE);
    f << " Should receive: " << msglen << std::endl;
    f << "Receive " << std::endl;
    buf = (char*)malloc(msglen * MPI::BYTE.Get_size());
    MPI::COMM_WORLD.Recv(buf, msglen, MPI::BYTE, status.Get_source(), status.Get_tag(), status);

    RigidBodyMessage b2(2);
    b2.print(f);

    f << "Unmarshalling Body:" << std::endl;
    {
        boost::iostreams::basic_array_source<char>                           device(buf, msglen);
        boost::iostreams::stream<boost::iostreams::basic_array_source<char>> s(device);
        boost::archive::binary_iarchive                                      ia(s);
        // read class state from archive
        ia >> b2;
        // archive and stream closed when destructors are called
    }

    b2.print(f);

    free(buf);

    f << "=========================================================================" << std::endl;
}
/*
void sendBodyMessageMPIBoost(std::ostream & f = std::cout){
    f << "Sending BodyMessage (MPI)" << std::endl;

    boost::mpi::communicator world;
    int my_rank=world.rank();

    usleep(100*my_rank);

        //Send normal Body

        std::string serial_str;
        // create class instance

        RigidBodyMessage b2(my_rank);
        b2.fillRandom();
        b2.print(f);



        //f << "Sending String: " << serial_str <<std::endl;
        world.send(0,0,b2);
//        boost::mpi::content c = boost::mpi::get_content(b);
//        boost::mpi::skeleton<MyRigidBody> s = boost::mpi::skeleton(b);
//        world.send(0,0,s);
//        world.send(0,0,c);
}


void receiveBodyMessageMPIBoost(int source, std::ostream & f = std::cout){

    boost::mpi::communicator world;

    f << "=========== Receving Body (MPI), ProcID," << source <<" ==============" << std::endl;

    RigidBodyMessage b2(3);
    b2.printAddress(f);

//    world.recv(source,0, boost::mpi::skeleton(b2));
//    boost::mpi::content c = boost::mpi::get_content(b2);
//    world.recv(source,0,c);
    world.recv(source,0,b2);

    b2.print(f);
    f << "========================================================================="<<std::endl;
}


void sendBodyMessageMPIBoost2(std::ostream & f = std::cout){
    f << "Sending BodyMessage (MPI)" << std::endl;

    boost::mpi::communicator world;
    int my_rank=world.rank();

    usleep(100*my_rank);

        //Send normal Body

        std::string serial_str;
        // create class instance

        MyRigidBody b2;
        fillBodyRandom(b2);
        printBody(b2,f);



        //f << "Sending String: " << serial_str <<std::endl;


        boost::mpi::skeleton_proxy<MyRigidBody> s = boost::mpi::skeleton(b2);
        world.send(0,0,s);
//
        boost::mpi::content c = boost::mpi::get_content(b2);
        world.send(0,0,c);


//        //Send another
//        RigidBodyMessage b5(5);
//        boost::mpi::content c2 = boost::mpi::get_content(b5);
//        world.send(0,0,c2);
}


void receiveBodyMessageMPIBoost2(int source, std::ostream & f = std::cout){

    boost::mpi::communicator world;

    f << "=========== Receving Body (MPI), ProcID," << source <<" ==============" << std::endl;

    MyRigidBody b2;
    printBody(b2,f);

    //boost::mpi::skeleton_proxy<RigidBodyMessage> s = boost::mpi::skeleton(b2);
    world.recv(source,0, boost::mpi::skeleton(b2));
//
//
    boost::mpi::content c = boost::mpi::get_content(b2);
    world.recv(source,0,c);
    printBody(b2,f);

//    RigidBodyMessage b4(4);
//    boost::mpi::content c2 = boost::mpi::get_content(b4);
//    world.recv(source,0,c2);
//    b4.print(f);


    f << "========================================================================="<<std::endl;
}

*/

int testSerializationRigidBodyMessageMPI(int argc, char** argv)
{
    int         my_rank;
    int         p;
    int         source;
    int         dest;
    int         tag = 0;
    MPI::Status status;

    MPI::Init(argc, argv);

    my_rank = MPI::COMM_WORLD.Get_rank();
    p       = MPI::COMM_WORLD.Get_size();

    std::srand(my_rank);

    // Make file
    std::stringstream name;
    name << "ProcessLogID_" << my_rank << ".txt";
    std::ofstream f(name.str());

    if (my_rank != 0)
    {
        sendBodyMessageMPI(f);
    }
    else
    { /* my_rank = 0 */
        for (source = 1; source < p; source++)
        {
            receiveBodyMessageMPI(source, f);
        }
    }

    MPI::Finalize();
}

void testWaitAll(int argc, char** argv)
{
    int         numtasks, rank, next, prev, buf[2], tag1 = 1, tag2 = 2;
    MPI_Request reqs[4];
    MPI_Status  stats[4];

    std::vector<MPI_Request> reqs2;

    std::vector<MPI_Request*> reqPtr;

    reqs2.reserve(10);
    for (int i = 0; i < 4; i++)
    {
        reqs2.push_back(NULL);
        reqPtr.push_back(&reqs2.back());
        std::cout << "INIT:" << *(reqPtr[i]) << "," << reqs2[i] << std::endl;
    }

    MPI_Init(&argc, &argv);
    MPI_Comm_size(MPI_COMM_WORLD, &numtasks);
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);

    prev = rank - 1;
    next = rank + 1;
    if (rank == 0)
        prev = numtasks - 1;
    if (rank == (numtasks - 1))
        next = 0;

    MPI_Irecv(&buf[0], 1, MPI_INT, prev, tag1, MPI_COMM_WORLD, reqPtr[0]);
    MPI_Irecv(&buf[1], 1, MPI_INT, next, tag2, MPI_COMM_WORLD, reqPtr[1]);

    MPI_Isend(&rank, 1, MPI_INT, prev, tag2, MPI_COMM_WORLD, reqPtr[2]);
    MPI_Isend(&rank, 1, MPI_INT, next, tag1, MPI_COMM_WORLD, reqPtr[3]);

    for (int i = 0; i < 4; i++)
    {
        std::cout << *reqPtr[i] << "," << reqs2[i] << std::endl;
    }

    std::cout << "Wait all" << std::endl;
    MPI_Waitall(4, &reqs2[0], stats);
    std::cout << "Wait finished" << std::endl;

    MPI_Finalize();
}

/*

int testSerializationRigidBodyMessageBoostMPI(int argc, char** argv) {

    boost::mpi::environment env(argc, argv);
    boost::mpi::communicator world;

    int my_rank = world.rank();

    std::srand(my_rank);

    //Make file
    std::stringstream name;
    name << "ProcessLogID_"<<my_rank<<".txt";
    std::ofstream f(name.str());

    //sendBodyMessageMPIBoost(f); //Just to debug
    if (my_rank != 0){
        sendBodyMessageMPIBoost(f);
    }
    else{ // my_rank = 0
        for (int source = 1; source < world.size(); source++){
             receiveBodyMessageMPIBoost(source,f);
        }
    }
}
int testSerializationRigidBodyMessageBoostMPI2(int argc, char** argv) {

    boost::mpi::environment env(argc, argv);
    boost::mpi::communicator world;

    int my_rank = world.rank();

    std::srand(my_rank);

    //Make file
    std::stringstream name;
    name << "ProcessLogID_"<<my_rank<<".txt";
    std::ofstream f(name.str());

    //sendBodyMessageMPIBoost(f); //Just to debug
    if (my_rank != 0){
        sendBodyMessageMPIBoost2(f);
    }
    else{ // my_rank = 0
        for (int source = 1; source < world.size(); source++){
             receiveBodyMessageMPIBoost2(source,f);
        }
    }
}

*/

// Crazy test to test MPI derived data types... hm...
/*
class MessageSubA{
public:
    double a1;

    MessageSubA(){
        a1 = -1;
    }

    void fillRandom(){
        a1 = std::rand();
    }

    void print(std::ostream & f = std::cout){
        f << "a1: " <<a1 <<std::endl;
    }

     MPI::Datatype getMPI_type(){
        int block_lengths[1];
        MPI::Aint displacements[1];
        MPI::Datatype types[1];

        MPI::Aint start_address;
        MPI::Aint address;

        block_lengths[0]  = 1;

        // Add the two types
        types[0]=MPI::DOUBLE;

        // Add displacements
        start_address = MPI::Get_address(this) ; // Message Address
        address = MPI::Get_address(&a1);
        displacements[0] = address - start_address;

        return MPI::Datatype::Create_struct(1,block_lengths, displacements,types);
     }
};

class MessageSubB{
public:
    double b1;
    int b2;

    MessageSubB(){
        b1 = -1;
        b2 = -1;
    }

    void fillRandom(){
        b1 = std::rand();
        b2 = std::rand();
    }

    void print(std::ostream & f = std::cout){
        f << "b1: " <<b1 <<std::endl;
        f << "b2: " <<b2 <<std::endl;
    }

    MPI::Datatype getMPI_type(){
        int block_lengths[2];
        MPI::Aint displacements[2];
        MPI::Datatype types[2];

        MPI::Aint start_address;
        MPI::Aint address;

        block_lengths[0] = block_lengths[1] = 1;

        // Add the two types
        types[0]=MPI::DOUBLE;
        types[1]=MPI::INT;

        // Add displacements
        start_address = MPI::Get_address(this) ; // Message Address
        address = MPI::Get_address(&b1);
        displacements[0] = address - start_address;
        address = MPI::Get_address(&b2);
        displacements[1] = address - start_address;

        return MPI::Datatype::Create_struct(2,block_lengths, displacements,types);
    }
};

class Message{
public:
    MessageSubA *a_ptr;
    MessageSubB *b_ptr;

    std::vector< MessageSubB *> bList;

    Message(){
        a_ptr = new MessageSubA();
        b_ptr = new MessageSubB();
    }

    Message(int count){
        a_ptr = new MessageSubA();
        b_ptr = new MessageSubB();

        for(int i=0;i < count ;i++){
            bList.push_back(new MessageSubB());
        }
    }

    ~Message(){
        delete a_ptr;
        delete b_ptr;
        for(int i=0;i < bList.size() ;i++){
           delete bList[i];
        }
    }

    void fillRandom(){
        a_ptr->fillRandom();
        b_ptr->fillRandom();

        for(int i=0;i < bList.size() ;i++){
            bList[i]->fillRandom();
        }
    }

    void print(std::ostream & f = std::cout){
        f << "BEGIN Message: ============================"<<std::endl;
        a_ptr->print(f);
        b_ptr->print(f);
        f << "END Message: ============================"<<std::endl;
    }

    MPI::Datatype getMPI_type(){
        int block_lengths[2];
        MPI::Aint displacements[2];
        MPI::Datatype types[2];

        MPI::Aint start_address;
        MPI::Aint address;

        block_lengths[0] = block_lengths[1] = 1;


        //get Type for MessageSubA
        MPI::Datatype typeSubA;
        typeSubA = a_ptr->getMPI_type();
        //get Type for MessageSubB
        MPI::Datatype typeSubB;
        typeSubB = b_ptr->getMPI_type();


        // Add the two types
        types[0]=typeSubA;
        types[1]=typeSubB;

        // Add displacements
        start_address=MPI::Get_address(this) ; // Message Address
        address=MPI::Get_address(a_ptr);
        displacements[0] = address - start_address;
        address=MPI::Get_address(b_ptr);
        displacements[1] = address - start_address;

       return MPI::Datatype::Create_struct(2,block_lengths, displacements,types);
    }
};

void sendSpecialMessageMPI(std::ostream & f = std::cout){
    f << "Sending Special Message (MPI)" << std::endl;

    int my_rank;
   my_rank = MPI::COMM_WORLD.Get_rank();

    usleep(100*my_rank);

    if(my_rank>3){

//        //Send Shit!
//        int tag = 0;
//        int dest = 0;
//        MPI::Request request;
//        char a = 3;
//        MPI::Isend(&a, 1, MPI::CHAR, dest, 0, MPI::COMM_WORLD,&request);

    }else{

        //Send

        Message mess;
        mess.fillRandom();

        mess.print(f);

        MPI::Datatype mpi_type;
        mpi_type = mess.getMPI_type();
        mpi_type.Commit();

        //MPI SENDING to Rank 0!
        int dest = 0;
        int tag = 0;
        MPI::Request request;
        request =  MPI::COMM_WORLD.Isend(&mess, 1,mpi_type, dest, tag); //&request);

    }
}


void receiveSpecialMessageMPI(int source, std::ostream & f = std::cout){


    f << "=========== Receving Sepcial Message (MPI), ProcID," << source <<" ==============" << std::endl;

    int tag = 0;
    MPI::Status status;


    // probing!!
    //int tag = 0;

//    f<<" Probe: "<<std::endl;
//    int flag = 0;
//
//    //Polling
//    while(1){
//        MPI::Iprobe(source, tag, MPI::COMM_WORLD,&flag, &status); // Blocks
//        f<<"Waiting for Process Source:  "<<source<<std::endl;
//        if(flag)
//            break;
//    }
//
//    f<<"Get Count: "<<std::endl;
//    MPI::Get_count(&status, MPI::CHAR, &msglen);
//    f << " Should receive: "<< msglen <<std::endl;


    Message mess;

        MPI::Datatype mpi_type;
        mpi_type = mess.getMPI_type();
        mpi_type.Commit();


    MPI::COMM_WORLD.Recv( &mess, 1, mpi_type, source, tag);

    mess.print(f);

    f << "========================================================================="<<std::endl;
}



int testSerializationSpecialMessageMPI(int argc, char** argv) {

    int my_rank;
    int p;
    int source;
    int dest;
    int tag = 0;
    MPI::Status status;

     MPI::Init(argc, argv);

    my_rank=MPI::COMM_WORLD.Get_rank();
    p = MPI::COMM_WORLD.Get_size();

    std::srand(my_rank);

    //Make file
    std::stringstream name;
    name << "ProcessLogID_"<<my_rank;
    std::ofstream f(name.str());


    if (my_rank != 0){
        sendSpecialMessageMPI(f);
    }
    else{ // my_rank = 0
        for (source = 1; source < p; source++){
             receiveSpecialMessageMPI(source,f);
        }
    }

    MPI::Finalize();
}
*/
