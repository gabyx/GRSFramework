#ifndef KdTreeTest_hpp
#define KdTreeTest_hpp

#include <pugixml/pugixml.hpp>

#include "GRSF/common/TypeDefs.hpp"
#include "GRSF/common/AssertionDebug.hpp"


#include "GRSF/dynamics/collision/geometry/AABB.hpp"

#include "GRSF/dynamics/general/KdTree.hpp"
#include "GRSF/dynamics/general/SerializationHelpersKdTree.hpp"
#include "GRSF/common/CPUTimer.hpp"

#include <boost/filesystem.hpp>
#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/device/back_inserter.hpp>

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

#include <boost/serialization/vector.hpp>
#include <boost/serialization/variant.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/version.hpp>


/**
* Composer which serilaizes one message: take care sending bytes and receiving them need to make sure that the same endianess is used in the network!
*/
class MessageBinarySerializer {
public:

    MessageBinarySerializer(std::size_t reserve_bytes) {
        m_buffer.reserve(reserve_bytes);
    }

    // Move copy constructor
    MessageBinarySerializer( MessageBinarySerializer && ref): m_buffer(std::move(ref.m_buffer)) {}

    // Move assignment
    MessageBinarySerializer & operator=(MessageBinarySerializer && ref) {
        m_buffer = std::move(ref.m_buffer);
        return *this;
    }

    void clear() { // Clears the buffer, but does not affect std::vector.capacity!
        m_buffer.clear();
    }

    /** Serialization of one object */
    template<typename T>
    MessageBinarySerializer & operator<<(const T & t) {
        m_buffer.clear(); //Clear serializable string, no allocation if we push less or equal as much into the string next time!
        boost::iostreams::back_insert_device<std::vector<char> > inserter(m_buffer);
        boost::iostreams::stream< boost::iostreams::back_insert_device<std::vector<char> > > s(inserter);
        boost::archive::binary_oarchive oa(s, boost::archive::no_codecvt | boost::archive::no_header);
        oa << t;
        s.flush();

        return *this;
    };

    /** Deserialization */
    template<typename T>
    MessageBinarySerializer & operator>>(T & t) {
        boost::iostreams::basic_array_source<char> device(data(), size());
        boost::iostreams::stream<boost::iostreams::basic_array_source<char> > s(device);
        boost::archive::binary_iarchive ia(s,  boost::archive::no_codecvt | boost::archive::no_header);
        ia >> t;

        return *this;
    };

    const  char * data() {
        return &m_buffer[0];
    }

    std::size_t size() {
        return m_buffer.size();
    }

    void resize(std::size_t bytes) {
        m_buffer.resize(bytes);
    }

    void reserve(std::size_t bytes) {
        m_buffer.reserve(bytes);
    }

private:
    std::vector<char> m_buffer;
};


namespace TestFunctions {

DEFINE_LAYOUT_CONFIG_TYPES

using  Vector3List = StdVecAligned<Vector3>;
using  Vector2List = StdVecAligned<Vector2>;

using  Matrix3Dyn = MatrixStatDyn<3>;
using  Matrix2Dyn = MatrixStatDyn<2>;



    Vector3List getPointsFromFile3D(std::string filePath) {

        std::ifstream file;            //creates stream myFile
        file.open(filePath.c_str());  //opens .txt file

        if (!file.is_open()) { // check file is open, quit if not
            ApproxMVBB_ERRORMSG("Could not open file: " << filePath)
        }

        PREC a,b,c;
        Vector3List v;
        while(file.good()) {
            file >> a>>b>>c;
            v.emplace_back(a,b,c);
        }
        file.close();
        return v;
    }

    Vector3List getPointsFromFile3DBin(std::string filePath) {

        std::ifstream file;            //creates stream myFile
        file.open(filePath.c_str(), std::ios::binary);  //opens .bin file

        if (!file.is_open()) { // check file is open, quit if not
            ApproxMVBB_ERRORMSG("Could not open file: " << filePath)
        }

        PREC a,b,c;
        Vector3List v;
        while(file.good()) {
            file.read((char*)&a,sizeof(PREC));
            file.read((char*)&b,sizeof(PREC));
            file.read((char*)&c,sizeof(PREC));
            v.emplace_back(a,b,c);
        }
        file.close();
        return v;
    }


    template<typename Tree>
    void saveTreeToXML(Tree & tree, const boost::filesystem::path & filePath = "./TopologyInfo_1.xml",
                       Vector3List * points = nullptr) {

        std::stringstream ss;

        // Open XML and write structure!
        pugi::xml_document dataXML;
        std::stringstream xml("<TopologyBuilder type=\"KdTree\" buildMode=\"\" time=\"0\" builtTime=\"0 \" >"
                                    "<Description>"
                                        "A_IK is tranformation matrix, which transforms points from frame K to frame I\n"
                                        "AABBList contains all AABBs from all ranks in frame I\n"
                                        "Time is the current simulation time\n"
                                        "BuiltTime is the elapsed time to build the topology\n"
                                        "AABBTree contains the tree without the leafs\n"
                                    "</Description>"
                                    "<AABBList />"
//                                    "<KdTree aligned=\"\">"
//                                        "<Root/>"
//                                        "<Leafs/>"
//                                        "<AABBTree/>"
//                                        "<A_IK/>"
//                                    "</KdTree>"
                                "</TopologyBuilder>");

        bool res = dataXML.load(xml);
        ASSERTMSG(res,"Could not load initial xml data file");

        // Write data

        using XMLNodeType = pugi::xml_node;
        XMLNodeType node;
        static const auto  nodePCData = pugi::node_pcdata;
        XMLNodeType root =  dataXML.child("TopologyBuilder");

        tree.appendToXML(root);

        if(points){
                std::stringstream ss;
                node = root.append_child("Points");
                for(auto & p : *points ) {
                    ss << p.transpose().format(MyMatrixIOFormat::SpaceSep) << std::endl;
                }
                node.append_child(nodePCData).set_value( ss.str().c_str() );
        }

        dataXML.save_file(filePath.c_str(),"    ");
    };

    template<typename Container>
    void dumpPoints(std::string filePath, Container & c) {

        std::ofstream l;
        l.open(filePath.c_str());
        if(!l.good()){
            ApproxMVBB_ERRORMSG("Could not open file: " << filePath << std::endl)
        }

        for(auto & v: c) {
            l << v.transpose().format(MyMatrixIOFormat::SpaceSep) << std::endl;
        }
        l.close();
    }

};

namespace Test{
    DEFINE_LAYOUT_CONFIG_TYPES


    void kdTreeTest() {

        //  Bad Split
        {
            using Tree = KdTree::Tree< KdTree::TreeTraits<> >;
            using SplitHeuristicType = Tree::SplitHeuristicType;
            using NodeDataType = Tree::NodeDataType;
            static const unsigned int Dimension = NodeDataType::Dimension;
            using PointListType = NodeDataType::PointListType;

            TestFunctions::Vector3List t = TestFunctions::getPointsFromFile3D("Points.txt");
            auto vec =  std::unique_ptr<PointListType>(new PointListType());
            AABB<Dimension> aabb;
            for(unsigned int i = 0; i<t.size(); ++i) {
                aabb+=t[i];
                vec->push_back(&t[i]);
            }
            std::cout << " 3d Points loaded: " << t.size() << std::endl;


            Tree tree;

            typename SplitHeuristicType::QualityEvaluator e(0.0, /* splitratio (maximized by MidPoint) */
                    1.0, /* pointratio (maximized by MEDIAN)*/
                    1.0);/* extentratio (maximized by MidPoint)*/
            PREC minExtent = 0.01;
            tree.initSplitHeuristic( std::initializer_list<SplitHeuristicType::Method> {
                                        //SplitHeuristicType::Method::MEDIAN,
                                        //SplitHeuristicType::Method::GEOMETRIC_MEAN,
                                        SplitHeuristicType::Method::MIDPOINT
                                    },
                                    21, minExtent,
                                    SplitHeuristicType::SearchCriteria::FIND_BEST,
                                    e,
                                    0.0, 0.0, 0.00);

            auto rootData = std::unique_ptr<NodeDataType>(new NodeDataType(vec->begin(),vec->end(), std::move(vec)));

            START_TIMER(start);
            tree.build(aabb,std::move(rootData), 500, 400);
            auto list = tree.buildLeafNeighbours(minExtent);
            STOP_TIMER_SEC(count,start );

            PREC avgNeighbours = 0;
            std::size_t minNeighbours = std::numeric_limits<std::size_t>::max();
            std::size_t maxNeighbours = std::numeric_limits<std::size_t>::lowest();

            std::ofstream f("neighborcountBadSplit.txt");
            for(auto & n: list) {
                f << n.second.size() << std::endl;
            }


            TestFunctions::saveTreeToXML(tree,"./BadSplit_1.xml", &t);

            std::cout << tree.getStatisticsString() << std::endl;

        }

        //  Good Split
        {
            using Tree = KdTree::Tree< KdTree::TreeTraits<> >;
            using SplitHeuristicType = Tree::SplitHeuristicType;
            using NodeDataType = Tree::NodeDataType;
            static const unsigned int Dimension = NodeDataType::Dimension;
            using PointListType = NodeDataType::PointListType;

            TestFunctions::Vector3List t = TestFunctions::getPointsFromFile3D("Points.txt");
            auto vec =  std::unique_ptr<PointListType>(new PointListType());
            AABB<Dimension> aabb;
            for(unsigned int i = 0; i<t.size(); ++i) {
                aabb+=t[i];
                vec->push_back(&t[i]);
            }
            std::cout << " 3d Points loaded: " << t.size() << std::endl;


            Tree tree;

            typename SplitHeuristicType::QualityEvaluator e(0.0, /* splitratio (maximized by MidPoint) */
                    1.0, /* pointratio (maximized by MEDIAN)*/
                    1.0);/* extentratio (maximized by MidPoint)*/
            PREC minExtent = 0.01;
            tree.initSplitHeuristic( std::initializer_list<SplitHeuristicType::Method> {
                                        SplitHeuristicType::Method::MEDIAN,
                                        SplitHeuristicType::Method::GEOMETRIC_MEAN,
                                        SplitHeuristicType::Method::MIDPOINT
                                    },
                                    20, minExtent,
                                    SplitHeuristicType::SearchCriteria::FIND_BEST,
                                    e,
                                    0.0, 0.0, 0.0);

            auto rootData = std::unique_ptr<NodeDataType>(new NodeDataType(vec->begin(),vec->end(), std::move(vec)));

            START_TIMER(start);
            tree.build(aabb,std::move(rootData), 500, 400);
            auto list = tree.buildLeafNeighbours(minExtent);
            STOP_TIMER_SEC(count,start );

            PREC avgNeighbours = 0;
            std::size_t minNeighbours = std::numeric_limits<std::size_t>::max();
            std::size_t maxNeighbours = std::numeric_limits<std::size_t>::lowest();

            std::ofstream f("neighborcountGoodSplit.txt");
            for(auto & n: list) {
                f << n.second.size() << std::endl;
            }


            TestFunctions::saveTreeToXML(tree,"./GoodSplit_2.xml", &t);

            std::cout << tree.getStatisticsString() << std::endl;

        }

//
//        {
//            using Tree = KdTree::Tree< KdTree::TreeTraits<> >;
//            using SplitHeuristicType = Tree::SplitHeuristicType;
//            using NodeType = Tree::NodeType;
//            using NodeDataType = Tree::NodeDataType;
//            static const unsigned int Dimension = NodeDataType::Dimension;
//            using PointListType = NodeDataType::PointListType;
//
//            TestFunctions::Vector3List t = TestFunctions::getPointsFromFile3D("KdTreeTest.txt");
//            t.resize(t.size());
//            auto vec =  std::unique_ptr<PointListType>(new PointListType());
//            AABB<Dimension> aabb;
//            for(unsigned int i = 0; i<t.size(); ++i) {
//                aabb+=t[i];
//                vec->push_back(&t[i]);
//            }
//            std::cout << " Points loaded: " << t.size() << std::endl;
//
//
//            Tree tree;
//
//            typename SplitHeuristicType::QualityEvaluator e(0.0, /* splitratio (maximized by MidPoint) */
//                                                            2.0, /* pointratio (maximized by MEDIAN)*/
//                                                            2.0);/* extentratio (maximized by MidPoint)*/
//            PREC minExtent = 0.0014;
//            tree.initSplitHeuristic( std::initializer_list<SplitHeuristicType::Method> {
//                                        SplitHeuristicType::Method::MEDIAN
//                                        /*SplitHeuristicType::Method::GEOMETRIC_MEAN,
//                                        SplitHeuristicType::Method::MIDPOINT*/
//                                    },
//                                    10, minExtent,
//                                    SplitHeuristicType::SearchCriteria::FIND_BEST,
//                                    e,
//                                    0.0, 0.0, 0.1);
//
//            auto rootData = std::unique_ptr<NodeDataType>(new NodeDataType(vec->begin(),vec->end(), std::move(vec)));
//            START_TIMER(start);
//            tree.build(aabb,std::move(rootData), 500, 384);
//            auto list = tree.buildLeafNeighbours(minExtent);
//            STOP_TIMER_SEC(count,start );
//
//            PREC avgNeighbours = 0;
//            std::size_t minNeighbours = std::numeric_limits<std::size_t>::max();
//            std::size_t maxNeighbours = std::numeric_limits<std::size_t>::lowest();
//            for(auto & n: list) {
//                //std::cout << "neighbours for idx: " << n.first;
//                avgNeighbours += n.second.size();
//                minNeighbours = std::min(minNeighbours,n.second.size());
//                maxNeighbours = std::max(maxNeighbours,n.second.size());
//    //                for(auto & i : n.second) {
//    //                    std::cout << int(i) << "," ;
//    //                }
//                //std::cout << std::endl;
//                //std::cout <<" count: " << n.second.size() << std::endl;
//            }
//            avgNeighbours /= list.size();
//
//
//            std::cout << "built time: " << count << " sec. " << std::endl;
//            std::cout << tree.getStatisticsString() << std::endl;
//            std::cout << "Avg. neighbours: " << avgNeighbours << std::endl;
//            std::cout << "Max. neighbours: " << maxNeighbours << std::endl;
//            std::cout << "Min. neighbours: " << minNeighbours << std::endl;
//
//            // Test all points for the corresponding leaf
//            START_TIMER(start2);
//            const NodeType * ptr;
//            for(unsigned int l=0;l<10;l++){
//                for(unsigned int i = 0; i<t.size(); ++i) {
//                    ptr = tree.getLeaf(t[i]);
//                }
//            }
//            STOP_TIMER_SEC(count2,start2);
//            std::cout << "point check time: " << count2 << ": " << ptr <<  std::endl;
//
//            TestFunctions::saveTreeToXML(tree,"./");
//        }
//
//
//        {
//            using Tree = KdTree::Tree< KdTree::TreeTraits<> >;
//            using SplitHeuristicType = Tree::SplitHeuristicType;
//            using NodeDataType = Tree::NodeDataType;
//            static const unsigned int Dimension = NodeDataType::Dimension;
//            using PointListType = NodeDataType::PointListType;
//
//            TestFunctions::Vector3List t = TestFunctions::getPointsFromFile3D("Lucy.txt");
//            auto vec =  std::unique_ptr<PointListType>(new PointListType());
//            AABB<Dimension> aabb;
//            for(unsigned int i = 0; i<t.size(); ++i) {
//                aabb+=t[i];
//                vec->push_back(&t[i]);
//            }
//            std::cout << " 3d Points loaded: " << t.size() << std::endl;
//
//
//            Tree tree;
//
//            typename SplitHeuristicType::QualityEvaluator e(0.0, /* splitratio (maximized by MidPoint) */
//                    2.0, /* pointratio (maximized by MEDIAN)*/
//                    1.0);/* extentratio (maximized by MidPoint)*/
//
//            PREC minExtent= 0.0001; // box extents are bigger than this!
//            tree.initSplitHeuristic( std::initializer_list<SplitHeuristicType::Method> {
//                                        SplitHeuristicType::Method::MEDIAN,/*
//                                        SplitHeuristicType::Method::GEOMETRIC_MEAN,
//                                        SplitHeuristicType::Method::MIDPOINT*/
//                                    },
//                                    20, 0.0001,
//                                    SplitHeuristicType::SearchCriteria::FIND_BEST, e,
//                                    0.0, 0.0, 0.1);
//
//            auto rootData = std::unique_ptr<NodeDataType>(new NodeDataType(vec->begin(),vec->end(), std::move(vec)));
//            START_TIMER(start);
//            tree.build(aabb,std::move(rootData), 500, 50000);
//            auto list = tree.buildLeafNeighbours(minExtent);
//            STOP_TIMER_SEC(count,start );
//
//            PREC avgNeighbours = 0;
//            std::size_t minNeighbours = std::numeric_limits<std::size_t>::max();
//            std::size_t maxNeighbours = std::numeric_limits<std::size_t>::lowest();
//            for(auto & n: list) {
//                //std::cout << "neighbours for idx: " << n.first << std::endl;
//                avgNeighbours += n.second.size();
//                minNeighbours = std::min(minNeighbours,n.second.size());
//                maxNeighbours = std::max(maxNeighbours,n.second.size());
//    //                for(auto & i : n.second) {
//    //                    std::cout << int(i) << "," ;
//    //                }
//                //std::cout << std::endl;
//                //std::cout <<" count: " << n.second.size() << std::endl;
//            }
//            avgNeighbours /= list.size();
//
//
//            std::cout << "built time: " << count << " sec. " << std::endl;
//            std::cout << tree.getStatisticsString() << std::endl;
//            std::cout << "Avg. neighbours: " << avgNeighbours << std::endl;
//            std::cout << "Max. neighbours: " << maxNeighbours << std::endl;
//            std::cout << "Min. neighbours: " << minNeighbours << std::endl;
//
//
//            TestFunctions::saveTreeToXML(tree,"./");
//
//            using Tree2 = KdTree::TreeSimple< KdTree::TreeSimpleTraits<
//                                            KdTree::NoData<>,
//                                            KdTree::NodeSimple
//                                            >
//                                       >;
//            START_TIMER(start2);
//            Tree2 tree2(tree);
//            STOP_TIMER_SEC(count2,start2);
//            std::cout << " copy time for TreeSimple: " << count2 << " sec. " << std::endl;
//
//
//        }
//
//        {
//            MatrixStatDyn<4> P(4,700);
//            P.setRandom();
//            using refType = decltype(P.col(0));
//            StdVecAligned<  refType > t;
//            for(int i=0; i < P.cols(); ++i){
//                t.emplace_back(P.col(i));
//            }
//
//
//            using PointListType = StdVecAligned<  refType * >;
//            using Traits = KdTree::TreeTraits< KdTree::PointData<4,PointListType> >;
//            using Tree = KdTree::Tree<  Traits >;
//            using SplitHeuristicType = Tree::SplitHeuristicType;
//            using NodeDataType = Tree::NodeDataType;
//
//
//            auto vec =  std::unique_ptr<PointListType>(new PointListType());
//            AABB<4> aabb;
//            for(unsigned int i = 0; i<t.size(); ++i) {
//                aabb+=t[i];
//                vec->push_back(&t[i]);
//            }
//            std::cout << " 4d Points loaded: " << t.size() << std::endl;
//
//
//            Tree tree;
//
//            typename SplitHeuristicType::QualityEvaluator e(0.0, /* splitratio (maximized by MidPoint) */
//                    2.0, /* pointratio (maximized by MEDIAN)*/
//                    1.0);/* extentratio (maximized by MidPoint)*/
//            tree.initSplitHeuristic(std::initializer_list<SplitHeuristicType::Method>
//                                    {SplitHeuristicType::Method::MIDPOINT},
//                                    20, 0.0001,
//                                    SplitHeuristicType::SearchCriteria::FIND_BEST, e,
//                                    0.0, 0.0, 0.1);
//
//            auto rootData = std::unique_ptr<NodeDataType>(new NodeDataType(vec->begin(),vec->end(), std::move(vec)));
//
//            tree.build(aabb,std::move(rootData), 50, 384);
//            std::cout << tree.getStatisticsString() << std::endl;
//            TestFunctions::saveTreeToXML(tree,"./");
//
//        }


//        {
//            std::cout << "Simulation Data =====================================================" << std::endl;
//                    // Build Tree ====================================================================
//                    using Tree = KdTree::Tree< KdTree::TreeTraits<> >;
//                    using SplitHeuristicType = Tree::SplitHeuristicType;
//                    using NodeDataType = Tree::NodeDataType;
//                    static const unsigned int Dimension = NodeDataType::Dimension;
//                    using PointListType = NodeDataType::PointListType;
//
//                    TestFunctions::Vector3List t = TestFunctions::getPointsFromFile3D("Points.txt");
//        //            TestFunctions::Vector3List t = TestFunctions::getPointsFromFile3D("Bunny.txt");
//
//                    auto vec =  std::unique_ptr<PointListType>(new PointListType());
//                    AABB<Dimension> aabb;
//                    for(unsigned int i = 0; i<t.size(); ++i) {
//                        aabb+=t[i];
//                        vec->push_back(&t[i]);
//                    }
//                    std::cout << " 3d Points loaded: " << t.size() << std::endl;
//
//
//                    Tree tree;
//
//                    typename SplitHeuristicType::QualityEvaluator e(0.0, /* splitratio (maximized by MidPoint) */
//                            0.0, /* pointratio (maximized by MEDIAN)*/
//                            1.0);/* extentratio (maximized by MidPoint)*/
//                    PREC minExtent = 5e-3;
//                    tree.initSplitHeuristic( std::initializer_list<SplitHeuristicType::Method> {
//                                                SplitHeuristicType::Method::MEDIAN,
//                                                /*SplitHeuristicType::Method::GEOMETRIC_MEAN,
//                                                SplitHeuristicType::Method::MIDPOINT*/
//                                            },
//                                            10, minExtent,
//                                            SplitHeuristicType::SearchCriteria::FIND_BEST,
//                                            e,
//                                            0.0, 0.0, 0.0);
//
//                    auto rootData = std::unique_ptr<NodeDataType>(new NodeDataType(vec->begin(),vec->end(), std::move(vec)));
//
//                    START_TIMER(start);
//                    tree.build(aabb,std::move(rootData), 50000, 5000);
//                    auto list = tree.buildLeafNeighboursAutomatic();
//                    STOP_TIMER_SEC(count,start );
//
//                    std::cout << "built time: " << count << " sec. " << std::endl;
//                    std::cout << tree.getStatisticsString() << std::endl;
//
//                    TestFunctions::saveTreeToXML(tree,"./");
//        }

    }

    void serializeKdTree(){

            std::cout << "NORMAL TEST =====================================================" << std::endl;
            // Build Tree ====================================================================
            using Tree = KdTree::Tree< KdTree::TreeTraits<> >;
            using SplitHeuristicType = Tree::SplitHeuristicType;
            using NodeDataType = Tree::NodeDataType;
            static const unsigned int Dimension = NodeDataType::Dimension;
            using PointListType = NodeDataType::PointListType;

            TestFunctions::Vector3List t = TestFunctions::getPointsFromFile3DBin("Points.bin");
//            TestFunctions::Vector3List t = TestFunctions::getPointsFromFile3D("Bunny.txt");

            auto vec =  std::unique_ptr<PointListType>(new PointListType());
            AABB<Dimension> aabb;
            for(unsigned int i = 0; i<t.size(); ++i) {
                aabb+=t[i];
                vec->push_back(&t[i]);
            }
            std::cout << " 3d Points loaded: " << t.size() << std::endl;


            Tree tree;

            typename SplitHeuristicType::QualityEvaluator e(0.0, /* splitratio (maximized by MidPoint) */
                    2.0, /* pointratio (maximized by MEDIAN)*/
                    2.0);/* extentratio (maximized by MidPoint)*/
            PREC minExtent = 5e-3;
            tree.initSplitHeuristic( std::initializer_list<SplitHeuristicType::Method> {
                                        SplitHeuristicType::Method::MEDIAN,
                                        /*SplitHeuristicType::Method::GEOMETRIC_MEAN,
                                        SplitHeuristicType::Method::MIDPOINT*/
                                    },
                                    10, minExtent,
                                    SplitHeuristicType::SearchCriteria::FIND_BEST,
                                    e,
                                    0.0, 0.0, 0.0);

            auto rootData = std::unique_ptr<NodeDataType>(new NodeDataType(vec->begin(),vec->end(), std::move(vec)));

            START_TIMER(start);
            tree.build(aabb,std::move(rootData), 50000, 384);
            auto list = tree.buildLeafNeighboursAutomatic();
            STOP_TIMER_SEC(count,start );

            std::cout << "built time: " << count << " sec. " << std::endl;
            std::cout << tree.getStatisticsString() << std::endl;

            TestFunctions::saveTreeToXML(tree,"./");

            // classify all points
            std::unordered_map<std::size_t , std::unordered_set<std::size_t> > origClasses;
            for(std::size_t i=0;i<t.size();++i){
                auto idx = tree.getLeaf(t[i])->getIdx();
                origClasses[idx].emplace(i);
            }

            using Tree2 = KdTree::TreeSimpleS<>;
            {
                auto & nodes = tree.getNodes();
                for(unsigned int s=0; s < nodes.size(); s++){
                    if(nodes[s]->getIdx() != s){
                        ERRORMSG("INDEX WRONG")
                    }
                }
            }

            // =============================================================================

            // Copy Tree ================================================
            std::cout << "COPY TEST =====================================================" << std::endl;
            Tree2 tree2(tree);

            {
             auto & nodes = tree2.getNodes();
                for(unsigned int s=0; s < nodes.size(); s++){
                    if(nodes[s]->getIdx() != s){
                        ERRORMSG("INDEX WRONG")
                    }
                    //std::cout << "node idx: " << s << " leaf: " << nodes[s]->isLeaf() << std::endl;
                }
            }
            std::cout << tree2.getStatisticsString() << std::endl;

            // check copy with original
            {

                auto & nodesOrig = tree.getNodes();
                for(auto * n: tree2.getNodes()){
                    auto idx = n->getIdx();

                    if(n->getSplitAxis() != nodesOrig[idx]->getSplitAxis()){
                        ERRORMSG("Orig not the same as serialized!")
                    }
                    if(n->getSplitPosition() != nodesOrig[idx]->getSplitPosition()){
                        ERRORMSG("Orig not the same as serialized!")
                    }

                    if((n->aabb().m_minPoint.array() != nodesOrig[idx]->aabb().m_minPoint.array()).any() ){
                        ERRORMSG("AABB for idx:" << n->getIdx() << "not the same!")
                    }
                    if((n->aabb().m_maxPoint.array() != nodesOrig[idx]->aabb().m_maxPoint.array()).any() ){
                        ERRORMSG("AABB for idx:" << n->getIdx() << "not the same!")
                    }

                    if(n->isLeaf()){
                        auto itO = nodesOrig[idx]->getBoundaries().begin();
                        for(auto * b: n->getBoundaries()){
                            if( b && *itO){
                                if(b->getIdx() != (*itO)->getIdx()){
                                    ERRORMSG("Boundary wrong for idx: " << n->getIdx() <<" new idx: " << b->getIdx() << "and orig: "
                                             << (*itO)->getIdx())
                                }
                            }else if( !( b==nullptr && *itO == nullptr )){
                                ERRORMSG("Boundary wrong for idx: " << n->getIdx()
                                         << b<< "or" << (*itO))
                            }
                            ++itO;
                        }
                    }
                }
            }


            // serialize this tree2 with boost ==============================================
            std::cout << "SERIALIZE TEST =====================================================" << std::endl;
            MessageBinarySerializer m(1000000);
            KdTreeSerializer<Tree2> ser(tree2);
            m << ser;

            Tree2 treeDes;
            KdTreeSerializer<Tree2> des(treeDes);
            m >> des;
            std::cout << treeDes.getStatisticsString() << std::endl;

            // classify points into leafs
            std::unordered_map<std::size_t , std::unordered_set<std::size_t> > serClasses;
            for(std::size_t i=0;i<t.size();++i){
                auto idx = treeDes.getLeaf(t[i])->getIdx();
                serClasses[idx].emplace(i);
            }

            // compare point classification
            for(auto & pa : serClasses){
                if(origClasses.find(pa.first) == origClasses.end()){
                    ERRORMSG("Serialized version leaf idx: " << pa.first << " not existing in orig!")
                }
                auto & origPoints = origClasses[pa.first];
                for(auto i : pa.second){
                    if(origPoints.find(i) == origPoints.end()){
                        ERRORMSG("Serialized version leaf idx: " << pa.first << "contains point idx: " << i << " but orig not!")
                    }
                }
            }

            // compare left and right
            auto & nodes = treeDes.getNodes();
            for(auto * n: nodes){

                if(! n->isLeaf()){

                    if( (n->leftNode()==nullptr || n->rightNode()==nullptr) ){
                        ERRORMSG("idx: " << n->getIdx() << "has not childs!")
                    }

                    auto idx =  n->leftNode()->getIdx();
                    if( nodes[idx] != n->leftNode() ){
                        ERRORMSG("Node: idx: " << idx << "not the same pointer at position nodes[idx]")
                    }
                    idx =  n->rightNode()->getIdx();
                    if( nodes[idx] != n->rightNode() ){
                        ERRORMSG("Node: idx: " << idx << "not the same pointer at position nodes[idx]")
                    }
                }else{
                    if( ! (n->leftNode()==nullptr && n->rightNode()==nullptr) ){
                        ERRORMSG("Leaf needs to have nullptr")
                    }
                }

            }

            auto leafs = tree2.getLeafs().size();

            // Test lowest Ancestor
            for(int i=0;i<100;i++){
                auto k = rand()%leafs;
                auto * a = tree2.getLeaf(rand()%leafs);
                auto * b = tree2.getLeaf(rand()%leafs);
                auto * n = tree2.getLowestCommonAncestor(a,b);
                if(n==nullptr){
                    ERRORMSG("Ancestor!")
                }
            }

            {
                // check data of serialization and original
                auto & nodesOrig = tree.getNodes();
                for(auto * n: nodes){
                    auto idx = n->getIdx();
                    if(n->getSplitAxis() != nodesOrig[idx]->getSplitAxis()){
                        ERRORMSG("Orig not the same as serialized!")
                    }

                    if(n->getSplitPosition() != nodesOrig[idx]->getSplitPosition()){
                        ERRORMSG("Orig not the same as serialized!")
                    }

                    auto itO = nodesOrig[idx]->getBoundaries().begin();

                    if(n->isLeaf()){
                        for(auto * b: n->getBoundaries()){
                            if( b && *itO){
                                if(b->getIdx() != (*itO)->getIdx()){
                                    ERRORMSG("Boundary wrong for idx: " << n->getIdx() <<" new idx: " << b->getIdx() << "and orig: "
                                             << (*itO)->getIdx())
                                }
                            }else if( !( b== nullptr && *itO == nullptr)){
                                ERRORMSG("Boundary wrong for idx: " << n->getIdx()
                                         << b<< "or" << (*itO))
                            }
                            ++itO;
                        }
                    }


                }
            }

            pugi::xml_document dataXML;
            treeDes.appendToXML(dataXML);
            dataXML.save_file("KdTreeResults.xml","    ");

        }
};




#endif // KdTreeTest_hpp
