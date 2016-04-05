// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include <iostream>
//#include <cassert>
//#include <memory>
//#include <type_traits>
//#include <utility>
//#include <chrono>
//#include <set>
//#include <map>
//#include <sstream>
//#include <vector>
//#include <fstream>
//#include <iterator>
//#include <string>
//#include <unordered_set>
//#include <initializer_list>
//#include <tuple>
//#include <type_traits>
#include <Eigen/Dense>
////#include <Eigen/Sparse>

//#include <limits>
//#include <cmath>
//
////
//#include <boost/filesystem.hpp>
//#include <boost/tuple/tuple.hpp>
//#include <boost/shared_ptr.hpp>
//#include <boost/variant.hpp>
//#include <boost/iterator/transform_iterator.hpp>
//#include <meta.hpp>

//#include <boost/type_traits.hpp>
//#include <boost/thread.hpp>
//#include <boost/timer/timer.hpp>

//#include "IteratorTest.hpp"
//#include "TemplateDeduction.hpp"
//#include "TemplateTemplateParameter.hpp"
//#include "ConstVariantTest.hpp"
//#include "MultiIndexContainerTest.hpp"
//#include "MortonKeyTest.hpp"
//#include "CopyConstructorTest.hpp"
//#include "FStreamBufferTest.hpp"
//#include "FStreamOpenCloseTest.hpp"
//#include "MoveSemantics.hpp"

//#include "GRSF/common/TypeDefs.hpp"
//#include "GRSF/dynamics/general/DynamicsSystem.hpp"

//#include "CompileTimeTable.hpp"



// #include "ProxProfile.hpp"

//#include "FrontBackBufferComplicated.hpp"

//#include "RangeTest.hpp"
//#include "RangeTest2.hpp"

//#include "CudaRefcounting.hpp"

//#include "RangeTest3.hpp"
//#include "MultiIndexMapTest.hpp"

//#include "SceneParserTest.hpp"

#include "StringParserTest.hpp"
//#include "StringConversionBenchmark.hpp"

//#include "LogicNodeTest.hpp"

//#include "LoggerTest.hpp"

//#include "GDiamTest.hpp"

//#include "ComputeMVBBTests.hpp"


//#include "TemplateStateTree.hpp"
//#include "LoggerTest.hpp"
//#include "ContactGraphTest2.hpp"

//#include "KdTreeTest.hpp"

//#include "AABBnD.hpp"
//#include "ProfileVectorBuffers.hpp"

//#include "CalculateExpectedMass.hpp"
//#include "StringFormatterTest.hpp"

//#include "LinearReusableStorageTest.hpp"

//#include "PointCloudTest.hpp"

//#include "GRSF/common/TypeDefs.hpp"

//#include "EigenSpeedTest.hpp"

//#include "GRSF/common/MetaHelper.hpp"

int  main( int  argc, char  ** argv ){



    //eigenSpeedTest();


    //linearReusableStorageTest();

    //pointCloudTest();

    /* initialize random seed: */


    // stringFormatterTest();

    //Test::serializeKdTree();
    //Test::kdTreeTest();

    //AABBnDTest();
//        calcualteExpectedMassBin();
//        calculateExpectedMass();
//        calculateExpectedMassSimple();
        //profileVectorBuffers();

        //convertSomeNumbers();
//    doBenchmark();
    stringTest();
//    doBenchmark();


//    SceneParser1::test();
//    SceneParser2::test();
    //runTest();
    //runRangeTest();
    //frontBackBufferTest();

    //proxProfile();

    //printCompileTimeTable();

    //fstreamBufferTest();
    //fstreamOpenCloseTest();

    //copyConstructorTest();
    //iteratorTestRun();
    //templateTemplateParamsTest();
    //constVariantTest();
    //multiIndexContainerTest();
    //mortonKeyTest();

    return 0;
};
