#include <iostream>
  #include <boost/function.hpp>
  #include <boost/bind.hpp>


using namespace std;

class A{
public:
  virtual void foo(){
    cout << "A" <<endl;
  }; 
};

template <typename T>
class B : public A{
public:
  void foo(){
    cout << "B" <<endl;
  }; 
};

int main(){

  A *a = new B<int>(); 
  a->foo();
  system("pause");
}