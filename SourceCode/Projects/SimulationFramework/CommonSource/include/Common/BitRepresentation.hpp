#ifndef BitRepresentation_hpp
#define BitRepresentation_hpp

#include <ostream>
#include <climits>
#include <bitset>

namespace BitRepresentation{

template<typename T>
class BitRepresentation;

template<typename T>
std::ostream & operator<<(std::ostream & s, const BitRepresentation<T> & r);


template<typename T>
class BitRepresentation{
	public:
    BitRepresentation(const T & a): m_a(a){}
private:

    friend std::ostream & operator<< <T>(std::ostream & s,const BitRepresentation<T> & r);

    const T & m_a;
};

template<typename T>
std::ostream & operator<<(std::ostream & s, const BitRepresentation<T> & r){
    const char* beg = reinterpret_cast<const char*>(&r.m_a);
    const char* end = beg + sizeof(T);

    while(beg != end)
        s << std::bitset<CHAR_BIT>(*(beg++)) << ' ';

    return s;
}


/**
* You can use this within an ostream output sequence  e.g std::cout << toBitsRepresentation(value) << std::endl;
* Maybe include : using BitRepresentation::operator<< ;
*/
template<typename T>
BitRepresentation<T> toBitsRepresentation(const T& a)
{
    return BitRepresentation<T>(a);
}

};

#endif
