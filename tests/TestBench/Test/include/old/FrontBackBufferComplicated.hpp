// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef FrontBackBuffer_hpp
#define FrontBackBuffer_hpp

#include <boost/shared_ptr.hpp>
#include <boost/type_traits.hpp>


/**
* @ingroup StatesAndBuffers
* @brief This is a class to store the front and back buffer pointers to the DynamicsState. This class is used in the timestepper.
*/

#include <typeinfo>
#include <type_traits>
#include "StripModifiers.hpp"
#include "StaticAssert.hpp"




template< typename TF, typename TB = TF, bool MakeSharedPtr = false>
class FrontBackBuffer;

template< typename TF, typename TB >
class FrontBackBuffer< TF, TB , false> {

public:

        // If <const * int , int&> --> this result in is_same< int , int >
        // GRSF_STATIC_ASSERT( (std::is_same< typename StripModifiers<TF>::type, typename StripModifiers<TB>::type >::value ) );

    template <typename T>
    struct MyRefTypes {
        typedef T Org;
        typedef T* Ptr;
        typedef const T & Con;
        typedef T& Ref;
        typedef const T& CRef;
        static inline Ptr getUnderlyingPtr(T& v) {
            return &v;
        }
        static Ref getRef(T& v) {
            return v;
        }
    };

        //Specialization for Reference
    template <typename T>
    struct MyRefTypes<T&> {
        typedef T Org;
        typedef T* Ptr;
        typedef T & Con;
        typedef T& Ref;
        typedef const T& CRef;
        static inline Ptr getUnderlyingPtr(T& v) {
            return &v;
        }
        static inline Ref getRef(T& v) {
            return v;
        }
    };

        //Specialization for const Reference
    template <typename T>
    struct MyRefTypes<const T&> {
        typedef T Org;
        typedef T* Ptr;
        typedef const T & Con;
        typedef const T& Ref;
        typedef const T& CRef;
        static inline Ptr getUnderlyingPtr(const T& v) {
            return &const_cast<T&>(v);
        }
        static inline Ref getRef(const T& v) {
            return v;
        }
    };

        //Specialization for const
    template <typename T>
    struct MyRefTypes<const T> {
        typedef T Org;
        typedef T* Ptr;
        typedef const T & Con;
        typedef const T& Ref;
        typedef const T& CRef;
        static inline Ptr getUnderlyingPtr(const T& v) {
            return &const_cast<T&>(v);
        }
        static inline Ref getRef(const T& v) {
            return v;
        }
    };

        //Specialization for pointers
    template <typename T>
    struct MyRefTypes<T*> {
        typedef T* Ptr;
        typedef T Org;
        typedef T* Con;
        typedef T& Ref;
        typedef T* const CRef;  //! note this is a pointer....
        static inline Ptr getUnderlyingPtr(T* v) {
            return v;
        }
        static inline Ref getRef(T* v) {
            return *v;
        }
    };

        //Specialization for const pointers
    template <typename T>
    struct MyRefTypes<const T*> {
        typedef T Org;
        typedef T* Ptr;
        typedef const T* Con;
        typedef const T& Ref;
        typedef const T* const CRef; //! note this is a pointer....
        static inline Ptr getUnderlyingPtr(const T* v) {
            return const_cast<T*>(v);
        }
        static inline Ref getRef(const T* v) {
            return *v;
        }
    };


    typedef typename MyRefTypes<TF>::Ref TFRef;
    typedef typename MyRefTypes<TF>::CRef TFCRef;
    typedef typename MyRefTypes<TF>::Con TFCon;
    typedef typename MyRefTypes<TF>::Org TFOrg;
        typedef typename MyRefTypes<TF>::Ptr TFPtr;

    typedef typename MyRefTypes<TB >::Ref TBRef;
    typedef typename MyRefTypes<TB >::CRef TBCRef;
    typedef typename MyRefTypes<TB >::Con TBCon;
    typedef typename MyRefTypes<TB >::Org TBOrg;
    typedef typename MyRefTypes<TB >::Ptr TBPtr;

    explicit FrontBackBuffer(TFCon  front, TBCon   back): m_Front(front), m_Back(back)
    {
                m_pBack = MyRefTypes<TB>::getUnderlyingPtr(m_Back);
                m_pFront = MyRefTypes<TF>::getUnderlyingPtr(m_Front);

    };


    ~FrontBackBuffer()
    {};

    TFRef getFront() {
        return *m_pFront;
    }
    TBRef getBack() {
        return *m_pBack;
    }

        void swap(){
                TFPtr temp = m_pFront;
                m_pFront = m_pBack;
                m_pBack = temp;
    }

private:



    TFPtr  m_pFront;       ///< The pointer to front buffer
    TBPtr  m_pBack;         ///< The pointer to back buffer

    TF m_Front;       ///< The front buffer
    TB m_Back;         ///< The back buffer
};


template< typename TF, typename TB >
class FrontBackBuffer< TF, TB , true> {

public:

        // If <const * int , int&> --> this result in is_same< int , int >
        // GRSF_STATIC_ASSERT( (std::is_same< typename StripModifiers<TF>::type, typename StripModifiers<TB>::type >::value ) );

    template <typename T>
    struct MyRefTypes {
        typedef T Org;
        typedef T* Ptr;
        typedef const T & Con;
        typedef T& Ref;
        typedef const T& CRef;
        static inline Ptr getUnderlyingPtr(T& v) {
            return &v;
        }
        static Ref getRef(T& v) {
            return v;
        }
    };

        //Specialization for Reference
    template <typename T>
    struct MyRefTypes<T&> {
        typedef T Org;
        typedef T* Ptr;
        typedef T & Con;
        typedef T& Ref;
        typedef const T& CRef;
        static inline Ptr getUnderlyingPtr(T& v) {
            return &v;
        }
        static inline Ref getRef(T& v) {
            return v;
        }
    };

        //Specialization for const Reference
    template <typename T>
    struct MyRefTypes<const T&> {
        typedef T Org;
        typedef T* Ptr;
        typedef const T & Con;
        typedef const T& Ref;
        typedef const T& CRef;
        static inline Ptr getUnderlyingPtr(const T& v) {
            return &const_cast<T&>(v);
        }
        static inline Ref getRef(const T& v) {
            return v;
        }
    };

        //Specialization for const
    template <typename T>
    struct MyRefTypes<const T> {
        typedef T Org;
        typedef T* Ptr;
        typedef const T & Con;
        typedef const T& Ref;
        typedef const T& CRef;
        static inline Ptr getUnderlyingPtr(const T& v) {
            return &const_cast<T&>(v);
        }
        static inline Ref getRef(const T& v) {
            return v;
        }
    };

        //Specialization for pointers
    template <typename T>
    struct MyRefTypes<T*> {
        typedef T* Ptr;
        typedef T Org;
        typedef T* Con;
        typedef T& Ref;
        typedef T* const CRef;  //! note this is a pointer....
        static inline Ptr getUnderlyingPtr(T* v) {
            return v;
        }
        static inline Ref getRef(T* v) {
            return *v;
        }
    };

        //Specialization for const pointers
    template <typename T>
    struct MyRefTypes<T*> {
        typedef T Org;
        typedef T* Ptr;
        typedef const T* Con;
        typedef const T& Ref;
        typedef const T* const CRef; //! note this is a pointer....
        static inline Ptr getUnderlyingPtr(const T* v) {
            return const_cast<T*>(v);
        }
        static inline Ref getRef(const T* v) {
            return *v;
        }
    };


    typedef typename MyRefTypes<TF>::Ref TFRef;
    typedef typename MyRefTypes<TF>::CRef TFCRef;
    typedef typename MyRefTypes<TF>::Con TFCon;
    typedef typename MyRefTypes<TF>::Org TFOrg;
        typedef typename MyRefTypes<TF>::Ptr TFPtr;

    typedef typename MyRefTypes<TB >::Ref TBRef;
    typedef typename MyRefTypes<TB >::CRef TBCRef;
    typedef typename MyRefTypes<TB >::Con TBCon;
    typedef typename MyRefTypes<TB >::Org TBOrg;
    typedef typename MyRefTypes<TB >::Ptr TBPtr;

    explicit FrontBackBuffer(const boost::shared_ptr<TF>  &front,
                              const boost::shared_ptr<TB>   & back):
        m_Front(front),
        m_Back(back)
    {

    };


    ~FrontBackBuffer()
    {};

    TFRef getFront() {
        return *m_Front;
    }
    TBRef getBack() {
        return *m_Back;
    }

    void swap(){
        typedef typename std::remove_const<TF>::type typeF;
        typedef typename std::remove_const<TB>::type typeB;

                boost::shared_ptr<typeF> tempF = boost::const_pointer_cast<typeF>(m_Front);
                boost::shared_ptr<typeB> tempB = boost::const_pointer_cast<typeB>(m_Back);
                tempF.swap(tempB);

                m_Front = tempF;
                m_Back = tempB;
                //m_Front.swap(m_Back);
    }

private:

    boost::shared_ptr<TF> m_Front;       ///< The front buffer
    boost::shared_ptr<TB> m_Back;         ///< The back buffer
};



#define CHECK_TEST( _assert_ , _checknr_ ) \
    if( ! (_assert_) ){ \
        std::cout << "Check "<< _checknr_ << " failed !" <<std::endl; \
        return -1; \
    }else{ \
        std::cout << "Check "<< _checknr_ << " succeded!" <<std::endl; \
    }


int frontBackBufferTest(){
    int CheckNr=0;

{

    int front=1;
    int back=2;

    CheckNr++;
    FrontBackBuffer< const int*, int & > buf1(&front, back);
    buf1.getBack() = 3; // change from 2 to 3
    // buf.getFront() = 5; NO! is const!
    buf1.swap();
    CHECK_TEST( buf1.getBack()==1 && buf1.getFront()==3 &&
                buf1.getBack()==front && buf1.getFront()==back, CheckNr);

    CheckNr++;
    buf1.getBack()=4;
    CHECK_TEST(buf1.getBack()==4 && buf1.getFront()==3 &&
       buf1.getBack()==front && buf1.getFront()==back, CheckNr);

    CheckNr++;
    buf1.getBack()=6;
    CHECK_TEST(buf1.getBack()==6 && buf1.getFront()==3 &&
       buf1.getBack()==front && buf1.getFront()==back, CheckNr);


    CheckNr++;
    buf1.swap();
    CHECK_TEST(buf1.getBack()==3 && buf1.getFront()==6 &&
               buf1.getBack()==back && buf1.getFront()==front, CheckNr);
}


{
    int front=1;
    int back=2;

    CheckNr++;
    FrontBackBuffer< int &, int * > buf1(front, &back);
    buf1.getBack() = 3; // change from 2 to 3
    buf1.getFront() = 5; // works
    buf1.swap();
    CHECK_TEST( buf1.getBack()==5 && buf1.getFront()==3 &&
                buf1.getBack()==front && buf1.getFront()==back, CheckNr);


    CheckNr++;
    buf1.getBack()=4;
    CHECK_TEST(buf1.getBack()==4 && buf1.getFront()==3 &&
       buf1.getBack()==front && buf1.getFront()==back, CheckNr);

    CheckNr++;
    buf1.getBack()=6;
    CHECK_TEST(buf1.getBack()==6 && buf1.getFront()==3 &&
       buf1.getBack()==front && buf1.getFront()==back, CheckNr);


    CheckNr++;
    buf1.swap();
    CHECK_TEST(buf1.getBack()==3 && buf1.getFront()==6 &&
               buf1.getBack()==back && buf1.getFront()==front, CheckNr);
}

{
    int front=1;
    int back=2;

    CheckNr++;
    FrontBackBuffer< int *, int * > buf1(&front, &back);
    buf1.getBack() = 3; // change from 2 to 3
    // buf.getFront() = 5; NO! is const!
    buf1.swap();
    CHECK_TEST( buf1.getBack()==1 && buf1.getFront()==3 &&
                buf1.getBack()==front && buf1.getFront()==back, CheckNr);


    CheckNr++;
    buf1.getBack()=4;
    CHECK_TEST(buf1.getBack()==4 && buf1.getFront()==3 &&
       buf1.getBack()==front && buf1.getFront()==back, CheckNr);

    CheckNr++;
    buf1.getBack()=6;
    CHECK_TEST(buf1.getBack()==6 && buf1.getFront()==3 &&
       buf1.getBack()==front && buf1.getFront()==back, CheckNr);


    CheckNr++;
    buf1.swap();
    CHECK_TEST(buf1.getBack()==3 && buf1.getFront()==6 &&
               buf1.getBack()==back && buf1.getFront()==front, CheckNr);
}


{

    int front = 1;
    int back= -1;
    FrontBackBuffer<int &, int > buf1(front, back);

    CheckNr++;
    buf1.getBack() = 2;
    buf1.getFront() = 3;
    CHECK_TEST(buf1.getBack()==2 && buf1.getFront()==3 &&
               buf1.getBack()!=back && buf1.getFront()==front, CheckNr);

    CheckNr++;
    buf1.swap();
    CHECK_TEST(buf1.getBack()==3 && buf1.getFront()==2 &&
               buf1.getBack()==front && buf1.getFront()!=back && buf1.getFront()!=front, CheckNr);
}

{

    int front = 1;
    int back= 2;
    FrontBackBuffer<int &, const int> buf1(front, back);
    //buf1.getBack() = 2; // IS CONST!!

    CheckNr++;
    buf1.getFront() = 4;
    buf1.swap();
    buf1.getFront() = 3;
    buf1.swap();

    CHECK_TEST(buf1.getBack()==3 && buf1.getFront()==4 &&
               buf1.getBack()!= back && buf1.getFront()==front && buf1.getFront()!=back, CheckNr);
}


{
    boost::shared_ptr<int> b_ptr(new int(3));
    boost::shared_ptr<int> f_ptr(new int(4));
//    b_ptr.swap(temp);

    FrontBackBuffer<int, int,true> buf1(f_ptr,b_ptr);

    CheckNr++;
    buf1.getBack()=5;
    buf1.getFront()=6;

    CHECK_TEST(buf1.getBack()==5 && buf1.getFront()==6 &&
               buf1.getBack()== *b_ptr && buf1.getFront()==*f_ptr, CheckNr);

    CheckNr++;
    buf1.swap();
    CHECK_TEST(buf1.getBack()==6 && buf1.getFront()==5 &&
               buf1.getBack()== *f_ptr && buf1.getFront()==*b_ptr, CheckNr);
}


    return 1;
}

#endif

