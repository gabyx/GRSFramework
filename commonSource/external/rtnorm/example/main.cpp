//  Example for using rtnorm
//  
//  Copyright (C) 2012 Guillaume Dollé, Vincent Mazet (LSIIT, CNRS/Université de Strasbourg)
//  Licence: GNU General Public License Version 2
//  see http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
//
//  Depends: LibGSL
//  OS: Unix based system


#include <iostream>
#include "rtnorm.hpp"


int main()
{
  using T = double;
  T a = 1;                 // Left bound
  T b = 9;                 // Right bound
  T mu = 2;                // Mean
  T sigma = 3;             // Standard deviation
  T s;  // Output argument of rtnorm
  int K = 1e9;                  // Number of random variables to generate

    T res =0;
  //--- generate and display the random numbers ---
  std::cout<<"# x p(x)"<<std::endl;
  
  std::mt19937_64 m(5);
  rtnorm::truncated_normal_distribution<T> tnormal(mu,sigma,a,b);
  
    std::cout<<a<<" "<<b<<std::endl;
    std::cout<<mu<<" "<<sigma<<std::endl;

  for(int k=0; k<K; k++)
  {
    s = tnormal(m);
    res += s;
  }
  std::cout << res << std::endl;

  return 0;
}

