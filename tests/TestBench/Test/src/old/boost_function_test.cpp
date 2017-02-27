// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <iostream>

using namespace std;

class A
{
    public:
    virtual int foo(int a) = 0;
};

class B : public A
{
    public:
    int foo(int a)
    {
        cout << "B Class:" << a << endl;
        return a;
    }
};

class C : public A
{
    public:
    void foo2()
    {
        cout << "C Class: void" << endl;
    };
    int foo(int a)
    {
        cout << "C Class:" << a << endl;
        return a;
    }
};

class button
{
    public:
    boost::function<void()> onClick;
};

class player
{
    public:
    void play(){};
    void stop(){};
};

button playButton, stopButton;
player thePlayer;

void connect()
{
    playButton.onClick = boost::bind(&player::play, &thePlayer);
    stopButton.onClick = boost::bind(&player::stop, &thePlayer);
}

int main()
{
    connect();

    // With no bind
    A* a = new B();

    boost::function<int(A*, int)> f;
    f = &A::foo;

    f(a, 3);

    // With bind
    A* a2 = new B();

    boost::function<int(int)> f2;
    boost::bind (&A::foo, a2, _1)(2);

    // Switch with some C
    int (A::*f3)(int) = &A::foo;
    A*       a3       = new C();
    A*       switcher;

    switcher = a;
    (switcher->*f3)(5);
    switcher = a3;
    (switcher->*f3)(5);

    C* c = new C();

    boost::function<void()> f4;
      f4 = boost::bind(;

    system("pause");
}