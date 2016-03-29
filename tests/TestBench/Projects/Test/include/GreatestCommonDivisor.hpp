// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GreatestCommonDivisor_hpp
#define GreatestCommonDivisor_hpp

    namespace MathFunctions {

        /** Greates common divisor */
        template<bool argsPositive = false,
                 typename T>
        typename std::enable_if<std::is_integral<T>::value,T >::type
        gcd2( T  a, T  b ) {

            if(!argsPositive) {
                a = std::abs( a );
                b = std::abs( b );
            }

            if( a == 0  ||  a == b ) {
                return  b;
            }
            if( b == 0 ) {
                return  a;
            }
            if( a > b ) {
                return  gcd2<true,T>( a % b, b );
            } else {
                return  gcd2<true,T>( a, b % a );
            }
        }

        /** Greates common divisor */
        template<bool argsPositive = false,
                 typename T>
        typename std::enable_if<std::is_integral<T>::value,T>::type
        gcd3( T  a, T  b, T  c ) {

            if(!argsPositive) {
                a = std::abs( a );
                b = std::abs( b );
                c = std::abs( c );
            }

            if   ( a == 0 )
                return  gcd2<true,T>( b, c );
            if   ( b == 0 )
                return  gcd2<true,T>( a, c );
            if   ( c == 0 )
                return  gcd2<true,T>( a, b );

            return  gcd2<true,T>( a, gcd2<true,T>( b, c ) );
        }
    }
#endif
