// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef PointCloudTest_hpp
#define PointCloudTest_hpp



#include <iostream>

#include <vector>
#include <ctime>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "GRSF/common/TypeDefs.hpp"
#include "GRSF/dynamics/general/MultiBodySimFile.hpp"
#include "GRSF/dynamics/buffers/DynamicsState.hpp"
#include "GRSF/common/Range.hpp"
#include "GRSF/common/CPUTimer.hpp"

#include "GRSF/dynamics/general/KdTree.hpp"

#include <pcl/visualization/cloud_viewer.h>

DEFINE_LAYOUT_CONFIG_TYPES

using Vector3Spec = MyMatrix<double>::Vector3;

struct MyPoint{
    Vector3Spec * m_p;
    unsigned int m_id;
};

struct MyPointGetter{
    static const Vector3Spec & get(const MyPoint & p){ return *(p.m_p);}
    static Vector3Spec & get(MyPoint & p){ return *(p.m_p);}
};

struct MyPoint2{
    Vector3Spec  m_p;
    unsigned int m_id;
};


struct MyPointGetter2{
    static const Vector3Spec & get(const MyPoint2 & p){ return p.m_p;}
    static Vector3Spec & get(MyPoint2 & p){ return p.m_p;}
};


void pointCloudTest() {


    MultiBodySimFile simFile;
    simFile.openRead("./SimStateTestEnd.sim", false);
    std::cout << simFile.getDetails(true).getString() << std::endl;
    if(!simFile.isGood()){
        GRSF_ERRORMSG("Could not open simfile" )
    }

    Range<unsigned int> r(std::make_pair((unsigned int)0,(unsigned int)simFile.getNSimBodies()));

    DynamicsState s(r.begin(),r.end());
    simFile >> s;
    std::cout << "Read: " << s.m_SimBodyStates.size() << "states!" << std::endl;
    AABB3d aabb;
    for( auto v : s.m_SimBodyStates){
        aabb+=v.m_q.head<3>();
    }
    Vector3 center = aabb.center();



    // PCL ===========================
    // neighbours
    int K = 10;

    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // Generate pointcloud data
    cloud->width = s.m_SimBodyStates.size();
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);

    for (size_t i = 0; i < cloud->points.size (); ++i) {
        auto v = s.m_SimBodyStates[i].m_q.head<3>();
        cloud->points[i].x = v(0);
        cloud->points[i].y = v(1);
        cloud->points[i].z = v(2);
    }
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> ("inputPoint.pcd", *cloud, false);
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    {
        START_TIMER(start)
        kdtree.setInputCloud (cloud);
        STOP_TIMER_SEC(count,start)
        std::cout << "KdTree PCL build: " << count <<  " sec." << std::endl;
    }


    // OUR ===========================

    using PointDataTraits = KdTree::DefaultPointDataTraits<3,Vector3Spec, MyPoint, MyPointGetter>;
    //using PointDataTraits = KdTree::DefaultPointDataTraits<3,Vector3,Vector3>;
    using Tree = KdTree::Tree< KdTree::TreeTraits<
                                    KdTree::PointData<PointDataTraits>
                                >
                            >;
    using SplitHeuristicType = Tree::SplitHeuristicType;
    using NodeDataType = Tree::NodeDataType;
    static const unsigned int Dimension = NodeDataType::Dimension;
    using PointListType = NodeDataType::PointListType;

    using PointListType = NodeDataType::PointListType;

    StdVecAligned<Vector3Spec> temp;
    for( auto v : s.m_SimBodyStates){
        temp.push_back(v.m_q.head<3>());
    }

    PointListType t;
    for(unsigned int i = 0; i<s.m_SimBodyStates.size(); ++i) {
        t.push_back(MyPoint{&temp[i],i});
        //t.push_back(temp[i]);
    }

     // Make kdTree;
    Tree tree;

    typename SplitHeuristicType::QualityEvaluator e(0.0, /* splitratio (maximized by MidPoint) */
                                                    2.0, /* pointratio (maximized by MEDIAN)*/
                                                    1.0);/* extentratio (maximized by MidPoint)*/

    PREC minExtent = 0.0; // box extents are bigger than this!
    PREC allowSplitAboveNPoints = 10;
    tree.initSplitHeuristic( std::initializer_list<SplitHeuristicType::Method> {
                                /*SplitHeuristicType::Method::MEDIAN,*/
                                /*SplitHeuristicType::Method::GEOMETRIC_MEAN,*/
                                SplitHeuristicType::Method::MIDPOINT
                            },
                            allowSplitAboveNPoints, minExtent,
                            SplitHeuristicType::SearchCriteria::FIND_FIRST, e,
                            0.0, 0.0, 0.1);

    auto rootData = std::unique_ptr<NodeDataType>(new NodeDataType(t.begin(),t.end()));
    unsigned int inf = std::numeric_limits<unsigned int>::max();
    START_TIMER(start);
    tree.build(aabb,std::move(rootData), inf /*max tree depth*/, inf /*max leafs*/);
    STOP_TIMER_SEC(count,start);
    std::cout << "KdTree My build: " << count << " sec. " << std::endl;
    std::cout << tree.getStatisticsString() << std::endl;


    // Bruteforce
    PointListType bForce(t);


//    // Check all points!
//    {
//         int loops = 1000000;
//         int slice = 7;
//         int nSlices = 8;
//         int start = slice * t.size()/nSlices;
//         int end = start + t.size()/nSlices;
//         for(int l = start; l < end; l++){
//
//            std::cout << "Checking point id: " << l << std::endl;
//            Vector3Spec randomP = *t[ l ].m_p;
//
//            // My KdTree
//
//            using KNNTraits = typename Tree::KNNTraits<>;
//            typename KNNTraits::PrioQueue kNearest(K);
//            typename KNNTraits::DistCompType & compDist = kNearest.getComperator();
//            kNearest.getComperator().m_ref = randomP;
//            START_TIMER(start2);
//            tree.getKNearestNeighbours<KNNTraits>(kNearest);
//            STOP_TIMER_SEC(count2,start2)
//            //std::cout << "kNearest My time: " << count2 << " sec. (point: " << kNearest.getComperator().m_ref.transpose() << ")" << std::endl;
//
//            auto sorterId = []( const typename KNNTraits::PrioQueue::value_type & a,
//                                                             const typename KNNTraits::PrioQueue::value_type & b  ){
//                                                                return a.m_id < b.m_id;
//                                                        };
//            // Bruteforce
//            std::nth_element(bForce.begin(),bForce.begin()+K,bForce.end(), compDist);
//
//
//            // sort id
//            std::sort( bForce.begin(),bForce.begin()+K , sorterId );
//            std::sort( kNearest.begin(),kNearest.end(), sorterId );
//
//            // Check if we have the same neighbours
//            if( kNearest.size()!=K){
//                std::cout << "Checking point id: " << l << std::endl;
//                GRSF_ERRORMSG("Did not find enough points")
//            }
//            auto it = kNearest.begin();
//            auto itB = bForce.begin();
//
//            for( ;it != kNearest.end(); ++it){
//
//                if (itB->m_id != it->m_id){
//
//                    std::cout  << "point bruteforce id: "
//                              << itB->m_id << " and my id: "
//                              << it->m_id << " do not match! " <<std::endl;
//
//                    std::sort( bForce.begin(),bForce.begin()+K , compDist );
//                    std::sort( kNearest.begin(),kNearest.end(), compDist );
//
//                    std::cout << " My : " << std::endl;
//                    int i=0;
//                    for(auto &p: kNearest){
//                        //std::cout << " : " << p.transpose() << " dist: " << compDist(p) <<  std::endl;
//                        std::cout <<"i:" << (i++) << " " << p.m_id << " : " << p.m_p->transpose() << " dist: " << compDist(p) <<  std::endl;
//                    }
//                    i = 0;
//                    std::cout << " Bruteforce : " << std::endl;
//                    for(auto g = bForce.begin(); g < bForce.begin()+K ; ++g ){
//                        //std::cout << " : " << p.transpose() << " dist: " << compDist(p) <<  std::endl;
//                        std::cout <<"i:" << (i++) << " " << g->m_id << " : " << g->m_p->transpose() << " dist: " << compDist(*g) <<  std::endl;
//                    }
//
//                    GRSF_ERRORMSG("Error CHECK")
//                }
//
//                itB++;
//            }
//
//        }
//     }


    {

// Create the filtering object
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
            START_TIMER(start)
              pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
              sor.setInputCloud (cloud);
              sor.setMeanK (K+1);
              sor.setStddevMulThresh (5.0);
              sor.filter (*cloud_filtered);
              sor.setNegative (true);
            STOP_TIMER_SEC(count,start)
            std::cout << "Filtering point cloud took: " << count << "sec. " << std::endl;
            sor.filter (*cloud_filtered);
            writer.write<pcl::PointXYZ> ("outlierPointsPCL1.pcd", *cloud_filtered, false);
        }

        {
            PointListType output;
            KdTree::NearestNeighbourFilter<PointDataTraits> f(K,5.0);
            START_TIMER(start3);
            std::cout << "input: " << t.size()  << std::endl;
            f.filter(t,aabb,output);
            STOP_TIMER_SEC(count3,start3)
            std::cout << "Filtered time: " << count3 << " sec. " <<  std::endl;
            std::cout << "Filtered: " << output.size() << std::endl;


            // Generate pointcloud data
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
            cloud->width = output.size();
            cloud->height = 1;
            cloud->points.resize (cloud->width * cloud->height);

            for (size_t i = 0; i < output.size (); ++i) {
                auto v = output[i];
                cloud->points[i].x = (*v.m_p)(0);
                cloud->points[i].y = (*v.m_p)(1);
                cloud->points[i].z = (*v.m_p)(2);
            }

            pcl::PCDWriter writer;
            writer.write<pcl::PointXYZ> ("outlierPointsMy.pcd", *cloud, false);
        }

        {
            using PointDataTraits = KdTree::DefaultPointDataTraits<3,Vector3Spec, MyPoint2, MyPointGetter2>;

            StdVecAligned<MyPoint2> temp2;
            unsigned int i = 0;
            for(auto v : temp){
                temp2.emplace_back(MyPoint2{v,i});
                i++;
            }

            StdVecAligned<MyPoint2> output;
            std::cout << "input: " << temp2.size()  << std::endl;
            KdTree::NearestNeighbourFilter<PointDataTraits> f(K,5.0);
            START_TIMER(start3);
            f.filter(temp2,aabb,output);
            STOP_TIMER_SEC(count3,start3)
            std::cout << "Filtered (direct points) time: " << count3 << " sec. " <<  std::endl;
            std::cout << "Filtered: " << output.size() << std::endl;
        }
        {
            PointListType output;
            std::cout << "input: " << t.size()  << std::endl;
            KdTree::NearestNeighbourFilter<PointDataTraits> f(K,5.0);
            START_TIMER(start3);
            f.filter(t,aabb,output);
            STOP_TIMER_SEC(count3,start3)
            std::cout << "Filtered time: " << count3 << " sec. " <<  std::endl;
            std::cout << "Filtered: " << output.size() << std::endl;
        }

        {
            // Check points
            for(auto & v: t){
                if( !(s.m_SimBodyStates[v.m_id].m_q.head<3>().array() == (*v.m_p).array()).all() ){
                    GRSF_ERRORMSG("Points int simbody and t no more the same!!??")
                }
            }
        }

        /**
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
            START_TIMER(start)
              pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
              sor.setInputCloud (cloud);
              sor.setMeanK (30);
              sor.setStddevMulThresh (2.0);
              sor.filter (*cloud_filtered);
              sor.setNegative (true);
            STOP_TIMER_SEC(count,start)
            std::cout << "Filtering point cloud took: " << count << "sec. " << std::endl;
              sor.filter (*cloud_filtered);
              writer.write<pcl::PointXYZ> ("outlierPoints2.pcd", *cloud_filtered, false);
        }
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
            START_TIMER(start)
              pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
              sor.setInputCloud (cloud);
              sor.setMeanK (50);
              sor.setStddevMulThresh (2.0);
              sor.filter (*cloud_filtered);
              sor.setNegative (true);
            STOP_TIMER_SEC(count,start)
            std::cout << "Filtering point cloud took: " << count << "sec. " << std::endl;
              sor.filter (*cloud_filtered);
              writer.write<pcl::PointXYZ> ("outlierPoints3.pcd", *cloud_filtered, false);
        }
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
            START_TIMER(start)
              pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
              sor.setInputCloud (cloud);
              sor.setMeanK (20);
              sor.setStddevMulThresh (3.0);
              sor.filter (*cloud_filtered);
              sor.setNegative (true);
            STOP_TIMER_SEC(count,start)
            std::cout << "Filtering point cloud took: " << count << "sec. " << std::endl;
              sor.filter (*cloud_filtered);
              writer.write<pcl::PointXYZ> ("outlierPoints4.pcd", *cloud_filtered, false);
        }
        //   pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
        //   viewer.showCloud(cloud_filtered);
        //   while (!viewer.wasStopped())
        //   {
        //   }
        */
    }

//    {
//
//            using PointDataTraits = KdTree::DefaultPointDataTraits<3,Vector3, MyPoint, MyPointGetter>;
//
//            using Tree = KdTree::Tree< KdTree::TreeTraits<
//                                            KdTree::PointData<PointDataTraits>
//                                        >
//                                    >;
//            using SplitHeuristicType = Tree::SplitHeuristicType;
//            using NodeDataType = Tree::NodeDataType;
//            static const unsigned int Dimension = NodeDataType::Dimension;
//            using PointListType = NodeDataType::PointListType;
//
//            StdVecAligned<Vector3> t;
//            for( auto v : s.m_SimBodyStates){
//                t.push_back(v.m_q.head<3>());
//            }
//            auto vec =  std::unique_ptr<PointListType>(new PointListType());
//            AABB<Dimension> aabb;
//            for(unsigned int i = 0; i<t.size(); ++i) {
//                aabb+=t[i];
//                vec->push_back(MyPoint{&t[i],i});
//            }
//            std::cout << " 3d Points loaded: " << t.size() << std::endl;
//
//
//             // Make kdTree;
//            Tree tree;
//
//            typename SplitHeuristicType::QualityEvaluator e(0.0, /* splitratio (maximized by MidPoint) */
//                                                            2.0, /* pointratio (maximized by MEDIAN)*/
//                                                            1.0);/* extentratio (maximized by MidPoint)*/
//            int kNeighboursMean = 2;
//            PREC minExtent = 0.0; // box extents are bigger than this!
//            PREC allowSplitAboveNPoints = 2;
//            tree.initSplitHeuristic( std::initializer_list<SplitHeuristicType::Method> {
//                                        /*SplitHeuristicType::Method::MEDIAN,
//                                        SplitHeuristicType::Method::GEOMETRIC_MEAN,*/
//                                        SplitHeuristicType::Method::MIDPOINT
//                                    },
//                                    allowSplitAboveNPoints, minExtent,
//                                    SplitHeuristicType::SearchCriteria::FIND_FIRST, e,
//                                    0.0, 0.0, 0.1);
//
//            auto rootData = std::unique_ptr<NodeDataType>(new NodeDataType(vec->begin(),vec->end()));
//            unsigned int inf = std::numeric_limits<unsigned int>::max();
//            START_TIMER(start);
//            tree.build(aabb,std::move(rootData), inf /*max tree depth*/, inf /*max leafs*/);
//            STOP_TIMER_SEC(count,start);
//            std::cout << "built time: " << count << " sec. " << std::endl;
//            std::cout << tree.getStatisticsString() << std::endl;
//
//            // KNearstContainer is a container adapter
//            using KNNTraits = typename Tree::KNNTraits<>;
//            typename KNNTraits::PrioQueue kNearest;
//            kNearest.reserve(kNeighboursMean);
//            typename KNNTraits::DistCompType & compDist = kNearest.getComperator();
//
//            kNearest.getComperator().m_ref = aabb.center();
//
//            START_TIMER(start2);
//            tree.getKNearestNeighbours<KNNTraits>(kNeighboursMean, kNearest);
//            STOP_TIMER_SEC(count2,start2)
//            std::cout << "k-nearest time: " << count2 << " sec. " << std::endl;
//
//            for(auto &p: kNearest){
//                std::cout <<p.m_id << " : " << p.m_p->transpose() << " dist: " << compDist(p) <<  std::endl;
//            }
//
//        }


    return;
}
#endif // PointCloudTest_hpp
