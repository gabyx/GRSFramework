#ifndef GRSF_Common_Exception_hpp
#define GRSF_Common_Exception_hpp

#include <stdexcept>
#include <exception>
#include <string>
#include <sstream>


class Exception : public std::runtime_error {
public:
    Exception(const std::stringstream & ss): std::runtime_error(ss.str()){};
private:

};

#define THROWEXCEPTION( message ) {std::stringstream ___s___ ; ___s___ << message << std::endl << " @ " << __FILE__ << " (" << __LINE__ << ")" << std::endl; throw Exception(___s___);}



#endif // Exception_hpp
