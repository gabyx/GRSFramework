/*
 *  CommonFunctions.cpp
 *
 *  Created by Gabriel NÃ¼tzi on 21.03.10.
 *  Copyright 2010 ETH. All rights reserved.
 *
 */

#include <CommonFunctions.hpp>

/// Includes =================================

/// ==========================================


using namespace std;
using namespace Eigen;


double Utilities::randd(double low, double high)
{
    return ((double)rand() / (double)RAND_MAX)  * (high-low) + low;
};

//
//template<>
//bool Utilities::stringToType<bool>(bool & t, const std::string& s)
//{
//   int a;
//   if(stringToType<int>(a, s)){
//      if(a){
//         t = true;
//         return true;
//      }else{
//         t = false;
//         return true;
//      }
//   }
//
//   if( s == "true" || s =="True" || s=="TRUE"){
//      t = true;
//      return true;
//   }
//   else if( s == "false" || s =="False" || s=="FALSE"){
//      t = false;
//      return true;
//   }
//
//   t = false;
//   return false;
//}


