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
#include <vector>
#include <Eigen/Dense>


class Foo
{
public:

  //EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //std::vector<Eigen::Vector2d> vec; // VECTOR2d has 2x 8byte = 16byte => vectorizable!! so 
  // Eigen::Vector4d v4; // need to be aligned
  //Eigen::Vector3d v3; // no need to align -> not multiple of 16byte

  std::vector<Eigen::Vector4d>
};



// THIS CODE CRASHED, IF: WIN32, and PREPROCESSOR DEFINE : EIGEN_VECTORIZE
// THIS WORKS id WIN64


int main()
{

#ifdef EIGEN_VECTORIZE 
  std::cout << "Vectorization on" <<std::endl;
#endif
  
  // Do this 1000 times till it hopefully crashed$
  // Dont delete till it gets into unalligned space in memory!
  for(int i =0 ; i<1000; i++){
    Foo *foo = new Foo;
    Foo *foo2 = new Foo;
 
      //foo->v4 = foo->v4 + foo2->v4;
  }
  
  //// DOES NOT Crash
  //for(int i =0 ; i<1000; i++){
  //  Foo *foo = new Foo;
  //  Foo *foo2 = new Foo;

  //  //foo->v3 = foo->v3 + foo2->v3;
  //}


  /*foo->vec.push_back(Eigen::Vector2d());
  foo2->vec.push_back(Eigen::Vector2d());
  foo->vec[0] = foo->vec[0] + foo2->vec[0];*/

  system("pause");
};