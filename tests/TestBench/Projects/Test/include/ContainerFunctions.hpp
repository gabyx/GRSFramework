#ifndef ContainerFunctions_hpp
#define ContainerFunctions_hpp

namespace ContainerFunctions {

/** Move elements to back of the container if Func returns true
* @return Iterator r  where the range [r,e] is the back part of the vector where Func returned true
*/
template<typename Iterator, typename Func>
Iterator moveToBackIf(Iterator b, Iterator  e, Func f) {

    if( !(std::distance(b,e)>0) ) {
        return b;
    }

    Iterator dest = b;
    while  ( b != e ) {
        if( f(*b) ) { // *b == 3  for example
            b++;
            continue;
        }

        if(dest!=b) {   // copy only if not at same position!
            *dest = *b; // copy  value to front (test is not true)
        }
        ++dest;
        ++b;
    }
    return dest;
}

/** Move element b of consecutive elements a,b to back of the container if Func(a,b) returns true
* @return Iterator r  where the range [r,e] is the back part of the vector where Func returned true
*/
template<typename Iterator, typename Func>
Iterator moveConsecutiveToBackIf(Iterator b, Iterator  e, Func f) {


    if( std::distance(b,e)<2 ) {
        return e;
    }

    Iterator comp = b;
    Iterator dest = ++b;
    while  ( b != e ) {
        if( f(*std::prev(dest), *b ) ) { // *b == std::next(b)  for example
            ++b;
            continue;
        }

        if(dest!=b) {   // copy only if not at same position!
            *dest = *b; // copy  value to front (test is not true)
        }
        //std::cout << *dest << std::endl;
        ++dest;
        ++b;
    }

    return dest;
}
};

#endif // ContainerFunctions_hpp
