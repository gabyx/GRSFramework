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

//#include "test/TypeDefs.hpp"
//#include "test/DynamicsSystem.hpp"

//#include "CompileTimeTable.hpp"



// #include "ProxProfile.hpp"

//#include "FrontBackBufferComplicated.hpp"

//#include "RangeTest.hpp"
//#include "RangeTest2.hpp"

//#include "CudaRefcounting.hpp"

//#include "RangeTest3.hpp"
//#include "MultiIndexMapTest.hpp"

//#include "SceneParserTest.hpp"

//#include "StringParserTest.hpp"
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

//#include "GRSF/Common/TypeDefs.hpp"

//#include "EigenSpeedTest.hpp"

#include "GRSF/Common/CommonFunctions.hpp"

int  main( int  argc, char  ** argv ){

    std::string s="1 2 3 4 5 6     ";
    Eigen::Quaternion<double> a;
    std::cout << Utilities::stringToType(a.coeffs(),s) << std::endl;

    std::cout << a.coeffs() << std::endl;

    int b;
    Utilities::stringToType(b,s);

    //eigenSpeedTest();


    //linearReusableStorageTest();

    //pointCloudTest();

    /* initialize random seed: */


    //stringFormatterTest();

    //Test::serializeKdTree();
    //Test::kdTreeTest();

    //AABBnDTest();
//        calcualteExpectedMassBin();
//        calculateExpectedMass();
//        calculateExpectedMassSimple();
        //profileVectorBuffers();

        //convertSomeNumbers();
//    doBenchmark();
//    stringTest();
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
