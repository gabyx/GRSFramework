// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef Singleton_hpp
#define Singleton_hpp

#include "Asserts.hpp"

/**
        \brief Singelton Class: You need to construct the singelton object (subclass of this singelton class) on the heap with new, once!
**/

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
        WARNINGMSG( ms_Singleton == 0 , "ms_Singleton == 0 : " << typeid(*ms_Singleton).name());
        ms_Singleton = static_cast< T* >( this );
    }
    ~Singleton( void ) {
        WARNINGMSG( ms_Singleton ,"ms_Singleton != 0 : " << typeid(*ms_Singleton).name());
        ms_Singleton = 0;
    }

    static T& getSingleton( void ) {
        WARNINGMSG( ms_Singleton ,"ms_Singleton != 0 : " << typeid(*ms_Singleton).name());
        return ( *ms_Singleton );
    }
    static T* getSingletonPtr( void ) {
        WARNINGMSG( ms_Singleton ,"ms_Singleton != 0 : " << typeid(*ms_Singleton).name());
        return ms_Singleton;
    }
};

template<typename T> T* Singleton<T>::ms_Singleton = 0;

};

#endif
