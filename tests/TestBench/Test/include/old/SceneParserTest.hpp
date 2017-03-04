// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef SceneParserTest_hpp
#define SceneParserTest_hpp

#include <iostream>
#include <type_traits>

namespace SceneParser1
{
using namespace std;

template <typename ParserTraits>
struct ModuleA
{
    using ParserType  = typename ParserTraits::ParserType;
    using DType       = typename ParserTraits::DType;
    using OptionsType = int;

    using ModuleBType    = typename ParserTraits::ModuleBType;
    using ModuleBOptions = typename ModuleBType::OptionsType;

    void foo()
    {
        std::cout << "ModuleA::foo: ParserType: " << typeid(ParserType).name() << std::endl;
        std::cout << "ModuleA::foo: ModuleBType: " << typeid(ModuleBType).name() << std::endl;
        std::cout << "ModuleA::foo: ModuleBOptions: " << typeid(ModuleBOptions).name() << std::endl;
    }
};

template <typename ParserTraits>
struct ModuleB
{
    using ParserType  = typename ParserTraits::ParserType;
    using DType       = typename ParserTraits::DType;
    using OptionsType = float;

    using ModuleAType    = typename ParserTraits::ModuleAType;
    using ModuleAOptions = typename ModuleAType::OptionsType;  // uncomment this!!

    void foo()
    {
        std::cout << "ModuleB::foo: ParserType: " << typeid(ParserType).name() << std::endl;
        std::cout << "ModuleB::foo: ModuleAType: " << typeid(ModuleAType).name() << std::endl;
        std::cout << "ModuleB::foo: ModuleAOptions: " << typeid(ModuleAOptions).name() << std::endl;
    }
};

// The PARSER TYPE TRAITS Struct!!
template <typename Parser, typename D>
struct ParserTraits
{
    using DType      = D;
    using ParserType = Parser;

    using ModuleAType = ModuleA<ParserTraits>;
    using ModuleBType = ModuleB<ParserTraits>;
};

template <typename D, typename Derived = void>
struct Parser
{
    using DType = D;

    // Inject the derived class as the parser class for the modules
    using DerivedType      = typename std::conditional<std::is_same<Derived, void>::value, Parser, Derived>::type;
    using ParserTraitsType = ParserTraits<DerivedType, DType>;

    using ModuleAType = typename ParserTraitsType::ModuleAType;
    using ModuleBType = typename ParserTraitsType::ModuleBType;

    using ModuleAOptions = typename ModuleAType::OptionsType;  // uncomment this!!
    using ModuleBOptions = typename ModuleBType::OptionsType;  // uncomment this!!

    virtual void foo()
    {
        std::cout << "Parser::foo" << std::endl;
        ModuleAType a;
        a.foo();
        ModuleBType b;
        b.foo();
    }
};

template <typename D>
struct ParserGUI : Parser<D, ParserGUI<D>>
{
    using Base = Parser<D, ParserGUI<D>>;

    void foo()
    {
        std::cout << "ParserGUI::foo" << std::endl;
        typename Base::ModuleAType a;
        a.foo();
        typename Base::ModuleBType b;
        b.foo();
    }
};

int test()
{
    std::cout << "SceneParser1" << std::endl;
    Parser<double> t;
    t.foo();

    ParserGUI<double> d;
    d.foo();

    ParserGUI<double> r;
    ParserGUI<double>::Base& base = r;
    base.foo();
}
};

namespace SceneParser2
{
using namespace std;

template <typename Parser>
struct Module
{
    using DType       = typename Parser::DType;
    using OptionsType = int;
};

template <typename D, typename Derived = void>
struct Parser
{
    using DType = D;

    using DerivedType = typename std::conditional<std::is_same<Derived, void>::value, Parser, Derived>::type;
    using ModuleType  = Module<DerivedType>;
    // using ModuleOptions = typename ModuleType::OptionsType; //uncomment this!!
};

template <typename D>
struct ParserGUI : Parser<D, ParserGUI<D>>
{
    using Base = Parser<D, ParserGUI<D>>;
};

int test()
{
    Parser<double> t;
    ParserGUI<double> d;
}
};

#endif  // SceneParserTest_hpp
