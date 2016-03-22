#include <iostream>
#include <vector>
#include <utility>
#include <Eigen/Dense>

class SomeClass
{
protected:
  void* m_pValue;
public:
  SomeClass(){}

  template < typename T > SomeClass(int a)
  {};

  template < typename T > operator T(void) { return (T)m_pValue; }
};

int main()
{
  SomeClass scInt(10);
  std::cout << (int)scInt << std::endl;
  SomeClass scChar('a');
  std::cout << (char)scChar << std::endl;
}