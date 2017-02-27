// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/shared_ptr.hpp>
#include <fstream>
#include <iostream>

using namespace std;

class Object
{
public:
    int a;
}

class Test
{
private:
    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar& BOOST_SERIALIZATION_NVP(a);
        ar& BOOST_SERIALIZATION_NVP(b);
        ar& BOOST_SERIALIZATION_NVP(c);
    }

    int   a;
    int   b;
    float c;

    boost::shared_ptr<Object> pObj;

public:
    inline Test(int a, int b, float c)
    {
        this->a = a;
        this->b = b;
        this->c = c;
        pObj    = boost::shared_ptr<Object>(Object());
        pObj->a =
    }
};

int main()
{
    std::ofstream ofs("filename.xml");

    Test* t = new Test(1, 2, 3.3);

    boost::archive::xml_oarchive oa(ofs);
    oa << BOOST_SERIALIZATION_NVP(t);

    Test*                        t2;
    boost::archive::xml_oarchive oa(ofs);
    oa >> t2;

    return 0;
}