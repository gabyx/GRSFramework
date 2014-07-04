#ifndef Exception_hpp
#define Exception_hpp

#include <stdexcept>
#include <exception>
#include <string>

#define THROWEXCEPTION( message ) { std::stringstream s; s << message << std::endl << " @ " << __FILE__ << " (" << __LINE__ << ")"; throw Exception(s); }

class Exception : public std::runtime_error {
public:
    Exception(const std::stringstream & ss): std::runtime_error(ss.str()){};
private:

};


#endif // Exception_hpp
