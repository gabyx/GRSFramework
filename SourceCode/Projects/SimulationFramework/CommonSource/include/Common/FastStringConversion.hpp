#ifndef FastStringConversion_hpp
#define FastStringConversion_hpp

/*
* According to http://tinodidriksen.com/2011/05/28/cpp-convert-string-to-double-speed/
* This naive implementation is the fastest method to convert! Below is a uncommented .hpp file with the benchmark!
*/


#define white_space(c) ((c) == ' ' || (c) == '\t')
#define valid_digit(c) ((c) >= '0' && (c) <= '9')


namespace StringConversion{

namespace detail{
// integral implementation
template<typename T>
typename std::enable_if< std::is_integral<T>::value ,bool>::type
toType(T & r , const char *p) {
    r = 0;

    // Skip leading white space, if any.
    while (white_space(*p) ) {
        p += 1;
    }

    // Get the sign!
    bool neg = false;
    if (*p == '-') {
        neg = true;
        ++p;
    }else if(*p == '+'){
        neg = false;
        ++p;
    }

    int c = 0; // counter to check how many numbers we got!
    while (valid_digit(*p)) {
        r = (r*10) + (*p - '0');
        ++p; ++c;
    }

    // FIRST CHECK:
    if(c==0){return false;} // we got no dezimal places: invalid number!
     // POST CHECK:
    // skip post whitespaces
    while( white_space(*p) ){
        ++p;
    }
    if(*p != '\0'){return false;} // if next character is not the terminating character: invalid number!

    if (neg) {r = -r;}
    return true;
}

// double,float implementation
template<typename T>
typename std::enable_if< std::is_floating_point<T>::value ,bool>::type
toType(T & r, const char *p) {

    // Skip leading white space, if any.
    while (white_space(*p) ) {
        p += 1;
    }

    r = 0.0;
    int c = 0; // counter to check how many numbers we got!

    // Get the sign!
    bool neg = false;
    if (*p == '-') {
        neg = true;
        ++p;
    }else if(*p == '+'){
        neg = false;
        ++p;
    }

    // Get the digits before decimal point
    while (valid_digit(*p)) {
        r = (r*10.0) + (*p - '0');
        ++p; ++c;
    }

    // Get the digits after decimal point
    if (*p == '.') {
        T f = 0.0;
        T scale = 1.0;
        ++p;
        while (*p >= '0' && *p <= '9') {
            f = (f*10.0) + (*p - '0');
            ++p;
            scale*=10.0;
            ++c;
        }
        r += f / scale;
    }

    // FIRST CHECK:
    if(c==0){return false;} // we got no dezimal places: invalid number!


    // Get the digits after the "e"/"E" (exponenet)
    if (*p == 'e' || *p == 'E'){
        unsigned int e = 0;

        bool negE = false;
        ++p;
        if (*p == '-') {
            negE = true;
            ++p;
        }else if(*p == '+'){
            negE = false;
            ++p;
        }
        // Get exponent
        c = 0;
        while (valid_digit(*p)) {
            e = (e*10) + (*p - '0');
            ++p; ++c;
        }

        // Check exponent limits!
//        if( !neg && e>std::numeric_limits<T>::max_exponent10 ){
//            e = std::numeric_limits<T>::max_exponent10;
//        }else if(e < std::numeric_limits<T>::min_exponent10 ){
//            e = std::numeric_limits<T>::max_exponent10;
//        }

        // SECOND CHECK:
        if(c==0){return false;} // we got no  exponent: invalid number!

        T scaleE = 1.0;
        // Calculate scaling factor.

        while (e >= 50) { scaleE *= 1E50; e -= 50; }
        //while (e >=  8) { scaleE *= 1E8;  e -=  8; }
        while (e >   0) { scaleE *= 10.0; e -=  1; }

        if (negE){
           r /= scaleE;
        }else{
           r *= scaleE;
        }
    }

    // POST CHECK:
    // skip post whitespaces
    while( white_space(*p) ){
        ++p;
    }
    if(*p != '\0'){return false;} // if next character is not the terminating character: invalid number!

    // Apply sign to number
    if(neg){ r = -r;}

    return true;
}
}; //detail




template<typename T>
inline bool toType(T & r, const std::string & s ){
    return detail::toType( r, s.c_str());
}

//Sepcial for boolean values
inline bool toType(bool & t, const std::string& s) {
        char a;
        if( detail::toType(a, s.c_str()) ) {
            if(a) {
                t = true; return true;
            } else {
                t = false; return true;
            }
        }

        if( s == "true") {
            t = true;
            return true;
        } else if( s == "false") {
            t = false;
            return true;
        }
        return false;
}

}; // StringConversion

#undef white_space
#undef valid_digit



// UNCOMMENTED BENCHMARK FILE!!

//#ifdef _MSC_VER
//    #define _SECURE_SCL 0
//    #define _CRT_SECURE_NO_DEPRECATE 1
//    #define WIN32_LEAN_AND_MEAN
//    #define VC_EXTRALEAN
//    #define NOMINMAX
//#endif
//
//#include <cstdlib>
//#include <cstdio>
//#include <cstring>
//#include <ctime>
//#include <cmath>
//#include <iostream>
//#include <string>
//#include <vector>
//#include <iomanip>
//#include <sstream>
//#include <boost/lexical_cast.hpp>
//#include <boost/spirit/include/qi.hpp>
//#include <boost/spirit/include/phoenix_core.hpp>
//#include <boost/spirit/include/phoenix_operator.hpp>
//#include "cycle.h"
//
//#include <bitset>
//
//static const size_t N = 100000;
//static const size_t R = 7;
//static const double scaleSize = 1000000.0;
//
//void PrintStats(std::vector<double> timings) {
//    double fastest = std::numeric_limits<double>::max();
//
//    std::cout << std::fixed << std::setprecision(2);
//    std::cout << "[";
//    for (size_t i = 1 ; i<timings.size()-1 ; ++i) {
//        fastest = std::min(fastest, timings[i]);
//        std::cout << timings[i] << ",";
//    }
//    std::cout << timings.back();
//    std::cout << "]";
//
//    double sum = 0.0;
//    for (size_t i = 1 ; i<timings.size() ; ++i) {
//        sum += timings[i];
//    }
//    double avg = sum / static_cast<double>(timings.size()-1);
//
//    sum = 0.0;
//    for (size_t i = 1 ; i<timings.size() ; ++i) {
//        timings[i] = pow(timings[i]-avg, 2);
//        sum += timings[i];
//    }
//    double var = sum/(timings.size()-2);
//    double sdv = sqrt(var);
//
//    std::cout << " with fastest " << fastest << ", average " << avg << ", stddev " << sdv;
//}
//
//double naive(const char *p) {
//    double r = 0.0;
//    bool neg = false;
//    if (*p == '-') {
//        neg = true;
//        ++p;
//    }
//    while (*p >= '0' && *p <= '9') {
//        r = (r*10.0) + (*p - '0');
//        ++p;
//    }
//    if (*p == '.') {
//        double f = 0.0;
//        int n = 0;
//        ++p;
//        while (*p >= '0' && *p <= '9') {
//            f = (f*10.0) + (*p - '0');
//            ++p;
//            ++n;
//        }
//        r += f / std::pow(10.0, n);
//    }
//    if (neg) {
//        r = -r;
//    }
//    return r;
//}
//
//#define white_space(c) ((c) == ' ' || (c) == '\t')
//#define valid_digit(c) ((c) >= '0' && (c) <= '9')
//
//template<typename T>
//bool naive(T & r, const char *p) {
//
//    // Skip leading white space, if any.
//    while (white_space(*p) ) {
//        p += 1;
//    }
//
//    r = 0.0;
//    int c = 0; // counter to check how many numbers we got!
//
//    // Get the sign!
//    bool neg = false;
//    if (*p == '-') {
//        neg = true;
//        ++p;
//    }else if(*p == '+'){
//        neg = false;
//        ++p;
//    }
//
//    // Get the digits before decimal point
//    while (valid_digit(*p)) {
//        r = (r*10.0) + (*p - '0');
//        ++p; ++c;
//    }
//
//    // Get the digits after decimal point
//    if (*p == '.') {
//        T f = 0.0;
//        T scale = 1.0;
//        ++p;
//        while (*p >= '0' && *p <= '9') {
//            f = (f*10.0) + (*p - '0');
//            ++p;
//            scale*=10.0;
//            ++c;
//        }
//        r += f / scale;
//    }
//
//    // FIRST CHECK:
//    if(c==0){return false;} // we got no dezimal places! this cannot be any number!
//
//
//    // Get the digits after the "e"/"E" (exponenet)
//    if (*p == 'e' || *p == 'E'){
//        unsigned int e = 0;
//
//        bool negE = false;
//        ++p;
//        if (*p == '-') {
//            negE = true;
//            ++p;
//        }else if(*p == '+'){
//            negE = false;
//            ++p;
//        }
//        // Get exponent
//        c = 0;
//        while (valid_digit(*p)) {
//            e = (e*10) + (*p - '0');
//            ++p; ++c;
//        }
//        // Check exponent limits!
//        if( !neg && e>std::numeric_limits<T>::max_exponent10 ){
//            e = std::numeric_limits<T>::max_exponent10;
//        }else if(e < std::numeric_limits<T>::min_exponent10 ){
//            e = std::numeric_limits<T>::max_exponent10;
//        }
//        // SECOND CHECK:
//        if(c==0){return false;} // we got no  exponent! this was not intended!!
//
//        T scaleE = 1.0;
//        // Calculate scaling factor.
//
//        while (e >= 50) { scaleE *= 1E50; e -= 50; }
//        //while (e >=  8) { scaleE *= 1E8;  e -=  8; }
//        while (e >   0) { scaleE *= 10.0; e -=  1; }
//
//        if (negE){
//           r /= scaleE;
//        }else{
//           r *= scaleE;
//        }
//    }
//
//    // POST CHECK:
//    // skip post whitespaces
//    while( white_space(*p) ){
//        ++p;
//    }
//    if(*p != '\0'){return false;} // if next character is not the terminating character
//
//    // Apply sign to number
//    if(neg){ r = -r;}
//
//    return true;
//}
//
//
//
//
//double atofNew (const char *p)
//{
//    int frac;
//    double sign, value, scale;
//
//    // Skip leading white space, if any.
//    while (white_space(*p) ) {
//        p += 1;
//    }
//
//    // Get sign, if any.
//    sign = 1.0;
//    if (*p == '-') {
//        sign = -1.0;
//        p += 1;
//
//    } else if (*p == '+') {
//        p += 1;
//    }
//
//    // Get digits before decimal point or exponent, if any.
//    for (value = 0.0; valid_digit(*p); p += 1) {
//        value = value * 10.0 + (*p - '0');
//    }
//
//    // Get digits after decimal point, if any.
//    if (*p == '.') {
//        double pow10 = 10.0;
//        p += 1;
//        while (valid_digit(*p)) {
//            value += (*p - '0') / pow10;
//            pow10 *= 10.0;
//            p += 1;
//        }
//    }
//
//    // Handle exponent, if any.
//    frac = 0;
//    scale = 1.0;
//    if ((*p == 'e') || (*p == 'E')) {
//        unsigned int expon;
//
//        // Get sign of exponent, if any.
//
//        p += 1;
//        if (*p == '-') {
//            frac = 1;
//            p += 1;
//
//        } else if (*p == '+') {
//            p += 1;
//        }
//
//        // Get digits of exponent, if any.
//
//        for (expon = 0; valid_digit(*p); p += 1) {
//            expon = expon * 10 + (*p - '0');
//        }
//        if (expon > 308) expon = 308;
//
//        // Calculate scaling factor.
//        while (expon >= 50) { scale *= 1E50; expon -= 50; }
//        while (expon >=  8) { scale *= 1E8;  expon -=  8; }
//        while (expon >   0) { scale *= 10.0; expon -=  1; }
//    }
//
//    // Return signed and scaled floating point result.
//    return sign * (frac ? (value / scale) : (value * scale));
//}
//
//int convertSomeNumbers(){
//
//std::string y = ".3";
//std::cout << naive(y.c_str()) <<std::endl;
//double d;
//bool r = naive(d,y.c_str());
//
//y = " 3.123e-3";
//r = naive(d,y.c_str());
//std::cout << r << ", " << d <<std::endl;
//
//y = "  -12.112e-12";
//r = naive(d,y.c_str());
//assert(r);
//std::cout << r << ", " << d <<std::endl;
//
//y = "   -1.3e-2c3a23";
//r = naive(d,y.c_str());
//assert(!r);
//std::cout << r << ", " << d <<std::endl;
//y = "a-1e-2";
//assert(!r);
//r = naive(d,y.c_str());
//std::cout << r << ", " << d <<std::endl;
//
//y = "123e";
//r = naive(d,y.c_str());
//assert(!r);
//std::cout << r << ", " << d <<std::endl;
//
//y = "e-5"; //needs to fail
//r = naive(d,y.c_str());
//std::cout << r << ", " << d <<std::endl;
//r = std::atof(y.c_str());
//std::cout << r << ", " << d <<std::endl;
//
//
//y = "1e-200";
//r = naive(d,y.c_str());
//std::cout << r << ", " << std::bitset<64>(*reinterpret_cast<long int *>(&d)) <<std::endl;
//
//y = "1e-308";
//r = naive(d,y.c_str());
//std::cout << r << ", " << d  <<std::endl;
//
////overflow
//y = "1e-309";
//r = naive(d,y.c_str());
//std::cout << r << ", " << d <<std::endl;
//float f;
//r = naive(f,y.c_str());
//std::cout << r << ", " << f <<std::endl;
//
//d = 1.0e308;
//std::cout << r << ", " << d  <<std::endl;
//}
//
//int doBenchmark() {
//    std::vector<std::string> nums;
//    nums.reserve(N);
//    for (size_t i=0 ; i<N ; ++i) {
//        std::string y;
//        if (i & 1) {
//            y += '-';
//        }
//        y += boost::lexical_cast<std::string>(i);
//        y += '.';
//        y += boost::lexical_cast<std::string>(i);
//        nums.push_back(y);
//    }
//
//    {
//        double tsum = 0.0;
//        std::vector<double> timings;
//        timings.reserve(R);
//        for (size_t r=0 ; r<R ; ++r) {
//            ticks start = getticks();
//            for (size_t i=0 ; i<nums.size() ; ++i) {
//                double x = naive(nums[i].c_str());
//                tsum += x;
//            }
//            ticks end = getticks();
//            double timed = elapsed(end, start) / scaleSize;
//            timings.push_back(timed);
//        }
//
//        std::cout << "naive: ";
//        PrintStats(timings);
//        std::cout << std::endl;
//        std::cout << tsum << std::endl;
//    }
//
//    {
//        double tsum = 0.0;
//        std::vector<double> timings;
//        timings.reserve(R);
//        for (size_t r=0 ; r<R ; ++r) {
//            ticks start = getticks();
//            for (size_t i=0 ; i<nums.size() ; ++i) {
//                double x;
//                naive(x,nums[i].c_str());
//                tsum += x;
//            }
//            ticks end = getticks();
//            double timed = elapsed(end, start) / scaleSize;
//            timings.push_back(timed);
//        }
//
//        std::cout << "naive (my impl.): ";
//        PrintStats(timings);
//        std::cout << std::endl;
//        std::cout << tsum << std::endl;
//    }
//    {
//        double tsum = 0.0;
//        std::vector<double> timings;
//        timings.reserve(R);
//        for (size_t r=0 ; r<R ; ++r) {
//            ticks start = getticks();
//            for (size_t i=0 ; i<nums.size() ; ++i) {
//                double x = atofNew(nums[i].c_str());
//                tsum += x;
//            }
//            ticks end = getticks();
//            double timed = elapsed(end, start) / scaleSize;
//            timings.push_back(timed);
//        }
//
//        std::cout << "atofNew: ";
//        PrintStats(timings);
//        std::cout << std::endl;
//        std::cout << tsum << std::endl;
//    }
//
//    {
//        double tsum = 0.0;
//        std::vector<double> timings;
//        timings.reserve(R);
//        for (size_t r=0 ; r<R ; ++r) {
//            ticks start = getticks();
//            for (size_t i=0 ; i<nums.size() ; ++i) {
//                double x = atof(nums[i].c_str());
//                tsum += x;
//            }
//            ticks end = getticks();
//            double timed = elapsed(end, start) / scaleSize;
//            timings.push_back(timed);
//        }
//
//        std::cout << "atof(): ";
//        PrintStats(timings);
//        std::cout << std::endl;
//        std::cout << tsum << std::endl;
//    }
//
//    {
//        double tsum = 0.0;
//        std::vector<double> timings;
//        timings.reserve(R);
//        for (size_t r=0 ; r<R ; ++r) {
//            ticks start = getticks();
//            for (size_t i=0 ; i<nums.size() ; ++i) {
//                double x = strtod(nums[i].c_str(), 0);
//                tsum += x;
//            }
//            ticks end = getticks();
//            double timed = elapsed(end, start) / scaleSize;
//            timings.push_back(timed);
//        }
//
//        std::cout << "strtod(): ";
//        PrintStats(timings);
//        std::cout << std::endl;
//        std::cout << tsum << std::endl;
//    }
//
//    {
//        double tsum = 0.0;
//        std::vector<double> timings;
//        timings.reserve(R);
//        for (size_t r=0 ; r<R ; ++r) {
//            ticks start = getticks();
//            for (size_t i=0 ; i<nums.size() ; ++i) {
//                double x = 0.0;
//                sscanf(nums[i].c_str(), "%lf", &x);
//                tsum += x;
//            }
//            ticks end = getticks();
//            double timed = elapsed(end, start) / scaleSize;
//            timings.push_back(timed);
//        }
//
//        std::cout << "sscanf(): ";
//        PrintStats(timings);
//        std::cout << std::endl;
//        std::cout << tsum << std::endl;
//    }
//
//    {
//        double tsum = 0.0;
//        std::vector<double> timings;
//        timings.reserve(R);
//        for (size_t r=0 ; r<R ; ++r) {
//            ticks start = getticks();
//            for (size_t i=0 ; i<nums.size() ; ++i) {
//                double x = boost::lexical_cast<double>(nums[i]);
//                tsum += x;
//            }
//            ticks end = getticks();
//            double timed = elapsed(end, start) / scaleSize;
//            timings.push_back(timed);
//        }
//
//        std::cout << "lexical_cast: ";
//        PrintStats(timings);
//        std::cout << std::endl;
//        std::cout << tsum << std::endl;
//    }
//
//    {
//        using boost::spirit::qi::double_;
//        using boost::spirit::qi::parse;
//        double tsum = 0.0;
//        std::vector<double> timings;
//        timings.reserve(R);
//        for (size_t r=0 ; r<R ; ++r) {
//            ticks start = getticks();
//            for (size_t i=0 ; i<nums.size() ; ++i) {
//                double x = 0.0;
//                char const *str = nums[i].c_str();
//                parse(str, &str[nums[i].size()], double_, x);
//                tsum += x;
//            }
//            ticks end = getticks();
//            double timed = elapsed(end, start) / scaleSize;
//            timings.push_back(timed);
//        }
//
//        std::cout << "spirit qi: ";
//        PrintStats(timings);
//        std::cout << std::endl;
//        std::cout << tsum << std::endl;
//    }
//
//    {
//        double tsum = 0.0;
//        std::vector<double> timings;
//        timings.reserve(R);
//        for (size_t r=0 ; r<R ; ++r) {
//            ticks start = getticks();
//            for (size_t i=0 ; i<nums.size() ; ++i) {
//                std::istringstream ss(nums[i]);
//                double x = 0.0;
//                ss >> x;
//                tsum += x;
//            }
//            ticks end = getticks();
//            double timed = elapsed(end, start) / scaleSize;
//            timings.push_back(timed);
//        }
//
//        std::cout << "stringstream: ";
//        PrintStats(timings);
//        std::cout << std::endl;
//        std::cout << tsum << std::endl;
//    }
//
//    {
//        double tsum = 0.0;
//        std::vector<double> timings;
//        timings.reserve(R);
//        for (size_t r=0 ; r<R ; ++r) {
//            ticks start = getticks();
//            std::istringstream ss;
//            for (size_t i=0 ; i<nums.size() ; ++i) {
//                ss.str(nums[i]);
//                ss.clear();
//                double x = 0.0;
//                ss >> x;
//                tsum += x;
//            }
//            ticks end = getticks();
//            double timed = elapsed(end, start) / scaleSize;
//            timings.push_back(timed);
//        }
//
//        std::cout << "stringstream reused: ";
//        PrintStats(timings);
//        std::cout << std::endl;
//        std::cout << tsum << std::endl;
//    }
//}


#endif // FastStringConversion
