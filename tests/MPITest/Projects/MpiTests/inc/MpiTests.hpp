
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>

#include <mpi.h>



void mpi_greetings(int argc, char** argv){

    int my_rank;
    int p;
    int source;
    int dest;
    int tag = 0;
    char message[100];
    MPI_Status status;

    MPI_Init(&argc, &argv);

    MPI_Comm_rank(MPI_COMM_WORLD, &my_rank);
    MPI_Comm_size(MPI_COMM_WORLD, &p);


    if ( my_rank != 0){
        sprintf(message, "Greetings from process %d!", my_rank);
        dest = 0;
        MPI_Send(message, strlen(message)+1, MPI_CHAR,
        dest, tag, MPI_COMM_WORLD);
    }
    else{ /* my_rank = 0 */
        for (source = 1; source < p; source++){
        MPI_Recv(message, 100, MPI_CHAR, source, tag, MPI_COMM_WORLD, &status);
        printf("%s\n", message);
        }
    }

    MPI_Finalize();

}

struct A{
    int a;
    int b;
    double *c;
    void fill(int v, int count=1){
        a=v;
        b=v;
        c=NULL;
    }
    void print(){
        std::cout <<"a: "<<a<<", b:"<<b<<std::endl;
    }
};
struct B{
    std::vector<int> a;
    void print(){
        for(int i=0;i<a.size();i++){
            std::cout << a[i] << ",";
        }
        std::cout << std::endl;
    }

    void fill(int v,int count){
        for(int i=0;i<count;i++){
            a.push_back(v);
        }
    }
};

struct C{
    std::vector<int> a;
    void print(){
        for(int i=0;i<a.size();i++){
            std::cout << a[i] << ",";
        }
        std::cout << std::endl;
    }

    void fill(int v,int count){
        for(int i=0;i<count;i++){
            a.push_back(v);
        }
    }
};

void TypeTest(){


    A t;
    std::cout<< "Size of t: " << sizeof(t) << std::endl;
    std::cout<< "Size of t.a: " << sizeof(t.a) << std::endl;
    std::cout<< "Size of t.b: " << sizeof(t.a) << std::endl;
    std::cout<< "Size of pointer t.c: " << sizeof(t.c) << std::endl;


    t.a=1;
    t.b=2;
    t.c = new double(3);
    std::cout<<"t contains: "<<t.a<<","<<t.b<<","<<*t.c<<std::endl;;
    std::cout<< "Adress of struct t: " << &t << std::endl;
    std::cout<< "Adress of pointer t.c: " << t.c << std::endl;

    //Cast to void pointer
    void * ptr = (void *)&t;
    std::cout<< "Adress of void ptr: " << ptr << std::endl;
    std::cout<< "Value of first entry in ptr: " << *((int*)(ptr+sizeof(int)*0)) << std::endl;
    std::cout<< "Value of second entry in ptr: " << *((int*)(ptr+sizeof(int)*1)) << std::endl;
    std::cout<< "Adress of third entry t.c: " <<(double**)(ptr+sizeof(int)*2)<< std::endl;
    double * ptrd = *(double**)(ptr+sizeof(int)*2);
    std::cout<< "Value of third entry in ptr: " << *ptrd << std::endl;

}


void TypeTestMPI(int argc, char** argv){

    typedef B AType;
    int my_rank;
    int p;
    int source;
    int dest;
    int tag = 0;
    MPI_Status status;

    C test; test.fill(0,10);
    std::cout << "test has: " << sizeof(test) <<" bytes"<<std::endl;
    std::cout << "test address: " << &test <<" bytes"<<std::endl;
    std::cout << "test.a address: " << &test.a <<" bytes"<<std::endl;
    std::cout << "test.a[0] address: " << &test.a[0] <<" bytes"<<std::endl;

    MPI_Init(&argc, &argv);

    MPI_Comm_rank(MPI_COMM_WORLD, &my_rank);
    MPI_Comm_size(MPI_COMM_WORLD, &p);


    int count = 2;

    if ( my_rank != 0){
        AType a;
        a.fill(my_rank,count);
        char * message = (char*) &a;
        dest = 0;
        std::cout << "Sending: " ; a.print();
        std::cout << "Sending: " << sizeof(a.a) <<" bytes"<<std::endl;
        MPI_Send(message, sizeof(a), MPI_CHAR, dest, tag, MPI_COMM_WORLD);
    }
    else{ /* my_rank = 0 */

        for (source = 1; source < p; source++){
            AType t; t.fill(0,count);
            char * message = (char*) &t;
            std::cout << "Receiving: " << sizeof(t) <<" bytes"<<std::endl;
            MPI_Recv(message, sizeof(t), MPI_CHAR, source, tag, MPI_COMM_WORLD, &status);
            std::cout << "Received from: "<<source<<" : ";
            t.print();
        }
    }

    MPI::Finalize();

}

void fileTestMPI(int argc, char** argv){

    int my_rank;
    int p;
    int source;
    int dest;
    int tag = 0;
    MPI_Status status;


    MPI_Init(&argc, &argv);

    MPI_Comm_rank(MPI_COMM_WORLD, &my_rank);
    MPI_Comm_size(MPI_COMM_WORLD, &p);


    std::ofstream f;
    std::stringstream s;
    s << "MpiMessageID_"<<my_rank<<".log";
    f.open(s.str());
    f << "Greetings from Process " << my_rank;


    MPI_Finalize();

}

