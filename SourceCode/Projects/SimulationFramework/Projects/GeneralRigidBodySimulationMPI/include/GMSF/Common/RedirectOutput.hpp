
#ifndef GMSF_Common_RedirectOutput_hpp
#define GMSF_Common_RedirectOutput_hpp


#include <iostream>


class RedirectOutputs
{
    std::ostream& myStream;
    std::streambuf *const myBuffer;
public:
    RedirectOutputs( std::ostream& lhs, std::ostream& rhs=std::cout )
        : myStream(rhs), myBuffer(myStream.rdbuf())
    {
        myStream.rdbuf(lhs.rdbuf());
    }

    ~RedirectOutputs () {
        myStream.rdbuf(myBuffer);
    }
};


#endif
