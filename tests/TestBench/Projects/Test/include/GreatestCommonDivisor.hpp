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
