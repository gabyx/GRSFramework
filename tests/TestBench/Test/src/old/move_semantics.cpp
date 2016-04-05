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