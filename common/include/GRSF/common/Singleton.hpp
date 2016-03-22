#ifndef GRSF_Common_Singleton_hpp
#define GRSF_Common_Singleton_hpp

#include "GRSF/Common/AssertionDebug.hpp"

/**
        \brief Singelton Class: You need to construct the singelton object (subclass of this singelton class) on the heap with new, or on the stack globally once!
**/


#define INSTANCIATE_UNIQUE_SINGELTON( type , name ) \
    INSTANCIATE_UNIQUE_SINGELTON_CTOR( type, name , () )


#define INSTANCIATE_UNIQUE_SINGELTON_CTOR( type, name , __ctor_args__ ) \
    auto name = std::unique_ptr< type >( new type __ctor_args__  );


namespace Utilities{

template <typename T>
class Singleton {
private:
    /** \brief Explicit private copy constructor. This is a forbidden operation.*/
    Singleton(const Singleton<T> &);

    /** \brief Private operator= . This is a forbidden operation. */
    Singleton& operator=(const Singleton<T> &);

protected:

    static T* ms_Singleton;

public:
    Singleton( void ) {
        ASSERTMSG( ms_Singleton == 0 , "ms_Singleton != 0 : " << typeid(*ms_Singleton).name());
        ms_Singleton = static_cast< T* >( this );
    }
    ~Singleton( void ) {
        ASSERTMSG( ms_Singleton ,"ms_Singleton == 0 : " << typeid(*ms_Singleton).name());
        ms_Singleton = 0;
    }

    static T& getSingleton( void ) {
        ASSERTMSG( ms_Singleton ,"ms_Singleton == 0 : " << typeid(*ms_Singleton).name());
        return ( *ms_Singleton );
    }


};

template<typename T> T* Singleton<T>::ms_Singleton = 0;

};

#endif
