
#include <iostream>
#include <utility>


using namespace std;

class A {
    int *data;
    public:
    A(int v) { data = new int; *data = v; }
    ~A() { delete data; }
   
      // copy semantics
  /*  A(A &a) { data = new int; *data = a.val(); }
    A& operator=(A& a) { *data = a.val(); return *this; }
   */
    // move semantics
    A(A &&a) : data(a.data) { 
       a.data = 0; 
       cout << "move"<<endl;
    }
    A& operator=(A&& a) {
       std::swap(data, a.data); return *this; 
    }

    int val() { return data == 0 ? 0 : *data; }
};


int main(){
 

   int a = 4;

   const int & b   = a;
   int c = int();


}