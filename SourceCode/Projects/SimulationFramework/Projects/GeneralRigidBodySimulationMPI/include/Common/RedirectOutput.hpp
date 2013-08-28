
#ifndef RedirectOutput_HPP
#define RedirectOutput_HPP


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
