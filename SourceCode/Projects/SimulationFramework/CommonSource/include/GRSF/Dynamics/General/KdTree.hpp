#ifndef GRSF_Dynamics_General_KdTree_hpp
#define GRSF_Dynamics_General_KdTree_hpp


#include <type_traits>
#include <initializer_list>
#include <memory>
#include <deque>
#include <tuple>
#include <utility>
#include <fstream>

#include <pugixml.hpp>
#include <meta.hpp>

#include "GRSF/Common/TypeDefs.hpp"
#include "GRSF/Common/AssertionDebug.hpp"
#include "GRSF/Common/StaticAssert.hpp"
#include "GRSF/Common/DemangleTypes.hpp"
#include "GRSF/Common/EnumClassHelper.hpp"
#include "GRSF/Common/CommonFunctions.hpp"
#include "GRSF/Common/CPUTimer.hpp"
#include "GRSF/Dynamics/Collision/Geometry/AABB.hpp"

namespace KdTree {

DEFINE_LAYOUT_CONFIG_TYPES

using DefaultPointListType = std::vector<Vector3 * >;


#define DEFINE_KDTREE_TYPES( __Traits__ )  \
    using NodeDataType = typename __Traits__::NodeDataType; \
    static const unsigned int Dimension = __Traits__::NodeDataType::Dimension; \
    using NodeType = typename __Traits__::NodeType; \
    using SplitHeuristicType = typename __Traits__::SplitHeuristicType; \
    //using SplitAxisType = typename __Traits__::SplitAxisType; \


/* Dereference is No-Op for non-pointer types*/
template<typename T>
struct PointDeref {
    inline static T & get(T & t) {
        return t;
    }
};
/* Dereference the pointer for pointer types*/
template<typename T>
struct PointDeref<T*> {
    inline static T & get(T* t) {
        return *t;
    }
};

/** Standart class for the node data type in the Tree */
template<unsigned int Dim = 3, typename TPointList = DefaultPointListType >
class PointData {
public:

    DEFINE_LAYOUT_CONFIG_TYPES

    static const unsigned int Dimension = Dim;

    using PointListType = TPointList; /** linear in memory!*/
    using value_type    = typename PointListType::value_type;
    //STATIC_ASSERTM(std::is_pointer<value_type>::value, "The value of the point list needs to be a pointer" )
    using iterator      = typename PointListType::iterator;
    using Deref = PointDeref<value_type>;

    /** Constructor for the root node, which owns the pointer list
     *  All child nodes don't set m_points internally
     */
    PointData(iterator begin, iterator end, std::unique_ptr<PointListType> points)
        : m_begin(begin), m_end(end), m_points(std::move(points)) {};


    /** Splits the data into two new node datas if the split heuristics wants a split */
    std::pair<PointData *, PointData * >
    split(iterator splitRightIt) const {
        // make left
        PointData * left  = new PointData(m_begin, splitRightIt );
        // make right
        PointData * right = new PointData(splitRightIt, m_end);
        return std::make_pair(left,right);

    }

    PREC getGeometricMean(unsigned int axis) {
        PREC ret = 0.0;

        for(auto & pPoint : *this) {
            ret += Deref::get(pPoint)(axis);
        }
        ret /= size();
        return ret;
    }

    iterator begin() {
        return m_begin;
    }

    iterator end() {
        return m_end;
    }

    std::size_t size() const {
        return std::distance(m_begin,m_end);
    }

    std::string getPointString() {
        std::stringstream ss;
        for(auto & pPoint : *this) {
            ss << Deref::get(pPoint).transpose().format(MyMatrixIOFormat::SpaceSep) << std::endl;
        }
        return ss.str();
    }

private:
    iterator m_begin,m_end; ///< The actual range of m_points which this node contains

    /** Constructor used internally, which is used not for the root node */
    PointData(iterator begin, iterator end)
        : m_begin(begin), m_end(end), m_points(nullptr) {};

    std::unique_ptr<PointListType> m_points;
};


/** Quality evaluator for the split heuristic */
class LinearQualityEvaluator {
public:

    LinearQualityEvaluator(PREC s = 1.0, PREC p = 1.0, PREC e = 1.0):
        m_weightSplitRatio(s), m_weightPointRatio(p), m_weightMinMaxExtentRatio(e) {}

    inline PREC setLevel(unsigned int level) {
        /* set parameters for tree level dependent (here not used)*/
    };

    inline PREC compute(const PREC & splitRatio, const PREC & pointRatio, const PREC & minMaxExtentRatio) {
        return  m_weightSplitRatio     *     2.0 * splitRatio
                + m_weightPointRatio     *     2.0 * pointRatio
                + m_weightMinMaxExtentRatio        * minMaxExtentRatio;
    }

private:
    /** Quality weights */
    PREC m_weightSplitRatio = 1.0;
    PREC m_weightPointRatio = 1.0;
    PREC m_weightMinMaxExtentRatio = 1.0;
};


template< typename Traits >
class SplitHeuristic;

template< typename TQualityEvaluator, typename Traits >
class SplitHeuristicPointData {
public:

    DEFINE_KDTREE_TYPES( Traits )
    DEFINE_LAYOUT_CONFIG_TYPES

    using PointListType = typename NodeDataType::PointListType;
    STATIC_ASSERT( (std::is_same< PointData<Dimension,PointListType>, NodeDataType >::value) )

    enum class Method : char { MIDPOINT, MEDIAN, GEOMETRIC_MEAN };

    /** Search Criterias how to find the best split
    * axis which fullfills the constraints, minExtent etc.
      FIND_BEST has not been implemented so far, because we shuffle the point array constantly while searching!
    */
    enum class SearchCriteria : char { FIND_FIRST, FIND_BEST };

    using QualityEvaluator = TQualityEvaluator;

    using SplitAxisType = typename NodeType::SplitAxisType;

    using PointDataType = NodeDataType;
    using value_type  = typename PointDataType::value_type;
    using iterator  = typename PointDataType::iterator;
    using Deref = typename PointDataType::Deref;

    SplitHeuristicPointData() : m_methods{Method::MIDPOINT}, m_searchCriteria(SearchCriteria::FIND_BEST) {
        for(SplitAxisType i = 0; i < Dimension; i++) {
            m_splitAxes.push_back(i);
        }
        resetStatistics();
    }

    void init(const std::initializer_list<Method>  m,
            unsigned int allowSplitAboveMinPoints = 100,
            PREC minExtent = 0.0,
            SearchCriteria searchCriteria = SearchCriteria::FIND_BEST,
            const QualityEvaluator & qualityEval = QualityEvaluator(),
            PREC minSplitRatio = 0.0, PREC minPointRatio = 0.0, PREC minExtentRatio = 0.0) {

        m_methods = m;
        if(m_methods.size()==0) {
            ERRORMSG("No methods for splitting given!")
        }
        m_allowSplitAboveMinPoints = allowSplitAboveMinPoints;
        m_minExtent = minExtent;
        if( m_minExtent < 0) {
            ERRORMSG("Minimal extent has wrong value!")
        }
        m_searchCriteria = searchCriteria;

        m_qualityEval = qualityEval;

        m_minSplitRatio = minSplitRatio;
        m_minPointRatio = minPointRatio;
        m_minExtentRatio = minExtentRatio;

        if( m_minSplitRatio < 0 || m_minPointRatio < 0 || m_minExtentRatio <0) {
            ERRORMSG("Minimal split ratio, point ratio or extent ratio have <0 values!");
        }

        resetStatistics();

    }

    void resetStatistics() {
        m_splitCalls = 0;
        m_tries = 0;
        m_splits = 0;

        m_avgSplitRatio = 0.0;
        m_avgPointRatio = 0.0;
        m_avgExtentRatio = 0.0;
    }

    std::string getStatisticsString() {
        std::stringstream s;
        s<<   "\t splits            : " << m_splits
                << "\n\t avg. split ratio  (0,0.5) : " << m_avgSplitRatio / m_splits
                << "\n\t avg. point ratio  [0,0.5] : " << m_avgPointRatio/ m_splits
                << "\n\t avg. extent ratio (0,1]   : " << m_avgExtentRatio / m_splits
                << "\n\t tries / calls     : " << m_tries << "/" << m_splitCalls << " = " <<(PREC)m_tries/m_splitCalls;
        return s.str();
    }

    /** Compute the split with the set heuristic and return the two new
    *   node data types if a split happened otherwise (nullptr)
    */
    std::pair<NodeDataType *, NodeDataType * >
    doSplit(NodeType * node) {

        ++m_splitCalls;

        auto * data = node->data();

        // No split for to little points or extent to small!
        if(data->size() <= m_allowSplitAboveMinPoints) {
            return std::make_pair(nullptr,nullptr);
        }


        // Sort split axes according to biggest extent
        m_extent = node->aabb().extent();

        auto biggest = [&](const SplitAxisType & a, const SplitAxisType & b) {
            return m_extent(a) > m_extent(b);
        };
        std::sort(m_splitAxes.begin(),m_splitAxes.end(),biggest);

        // Cycle through each method and check each axis
        unsigned int tries = 0;

        std::size_t nMethods = m_methods.size();
        std::size_t methodIdx = 0;
        std::size_t axisIdx = 0;
        m_found = false;
        m_bestQuality = std::numeric_limits<PREC>::lowest();
        //std::cout << "start: " << std::endl;
        while(((m_searchCriteria == SearchCriteria::FIND_FIRST && !m_found) ||
                m_searchCriteria == SearchCriteria::FIND_BEST )
                && methodIdx < nMethods) {

            //std::cout << "method " << methodIdx<<std::endl;
            m_wasLastTry = false;
            m_method = m_methods[methodIdx];
            m_splitAxis = m_splitAxes[axisIdx];

            // skip all axes which are smaller or equal than 2* min Extent
            // -> if we would split, left or right would be smaller or equal to min_extent!
            if( m_extent(m_splitAxis) > 2.0* m_minExtent ) {

                // Check split
                if( computeSplitPosition(node)) {

                    // check all min ratio values (hard cutoff)
                    if( m_splitRatio  >= m_minSplitRatio  &&
                            m_pointRatio  >= m_minPointRatio  &&
                            m_extentRatio >= m_minExtentRatio) {
                        //  if quality is better, update
                        updateSolution();
                        //std::cout << "Axis: " << axisIdx << "," << int(m_splitAxis)<< " q: " << m_quality << std::endl;
                        m_found = true;

                    }

                }

                ++tries;
            } else {
                std::cout << "cannot split -> extent " << std::endl;
            }


            // next axis
            if( ++axisIdx ==  Dimension) {
                axisIdx = 0;
                ++methodIdx;
            }

        }
        m_tries += tries;

        if(m_found) {

            // take best values
            // finalize sorting (for all except (lastTry true and method was median) )
            if( !(m_wasLastTry && m_bestMethod==Method::MEDIAN) ) {

                switch(m_bestMethod) {
                case Method::GEOMETRIC_MEAN:
                case Method::MIDPOINT: {
                    auto leftPredicate = [&](const value_type & a) {
                        return Deref::get(a)(m_bestSplitAxis) < m_bestSplitPosition;
                    };
                    m_bestSplitRightIt = std::partition(data->begin(), data->end(), leftPredicate );
                    break;
                }
                case Method::MEDIAN: {
                    auto less = [&](const value_type & a, const value_type & b) {
                        return Deref::get(a)(m_bestSplitAxis) < Deref::get(b)(m_bestSplitAxis);
                    };
                    std::nth_element(data->begin(), m_bestSplitRightIt, data->end(), less );
                    m_bestSplitPosition = Deref::get(*(m_bestSplitRightIt))(m_bestSplitAxis);
                    auto leftPredicate = [&](const value_type & a) {
                        return Deref::get(a)(m_bestSplitAxis) < m_bestSplitPosition;
                    };
                    m_bestSplitRightIt = std::partition(data->begin(),m_bestSplitRightIt, leftPredicate);
                    break;
                }
                }
            }

            computeStatistics();

            // Finally set the position and axis in the node!
            node->setSplitPosition(m_bestSplitPosition);
            node->setSplitAxis(m_bestSplitAxis);
            return data->split(m_bestSplitRightIt);
        }



        return std::make_pair(nullptr,nullptr);
    };



private:



    /** Compute the split position and maybe the splitAxis */
    bool computeSplitPosition(NodeType * node,
            unsigned int tries = 1) {

        auto * data = node->data();
        ASSERTMSG( data, "Node @  " << node << " has no data!")

        auto & aabb = node->aabb();
        switch(m_method) {
        case Method::MIDPOINT: {
            m_splitPosition = 0.5*(aabb.m_minPoint(m_splitAxis) + aabb.m_maxPoint(m_splitAxis));

            if(!checkPosition(aabb)) {
                return false;
            }
            // Compute bodyRatio/splitRatio and quality
            m_splitRatio = 0.5;
            m_extentRatio = computeExtentRatio(aabb);
            m_pointRatio = computePointRatio(data);
            m_quality = m_qualityEval.compute(m_splitRatio,m_pointRatio,m_extentRatio);

            break;
        }
        case Method::MEDIAN: {
            auto beg = data->begin();
            m_splitRightIt = beg + data->size()/2; // split position is the median!

            auto less = [&](const value_type & a, const value_type & b) {
                return Deref::get(a)(m_splitAxis) < Deref::get(b)(m_splitAxis);
            };
            // [5, 5, 1, 5, 6, 7, 9, 5] example for [beg ... end]
            // partition such that:  left points(m_splitAxis) <= nth element (splitRightIt) <= right points(m_splitAxis)
            std::nth_element(beg, m_splitRightIt, data->end(), less );
            m_splitPosition = Deref::get(*(m_splitRightIt))(m_splitAxis); // TODO make transform iterator to avoid Deref::geterence here!

            if(!checkPosition(aabb)) {
                return false;
            }


            // e.g [ 5 5 5 1 5 5 6 9 7 ]
            //               ^median, splitPosition = 5;

            // move left points which are equal to nth element to the right!

            auto leftPredicate = [&](const value_type & a) {
                return Deref::get(a)(m_splitAxis) < m_splitPosition;
            };
            m_splitRightIt = std::partition(beg,m_splitRightIt, leftPredicate);
            // it could happen that the list now looks [1 5 5 5 5 5 6 9 7]
            //                                            ^splitRightIt

            // Compute bodyRatio/splitRatio and quality
            m_splitRatio = computeSplitRatio(aabb);
            m_extentRatio = computeExtentRatio(aabb);
            m_pointRatio = (PREC)std::distance(beg,m_splitRightIt) / std::distance(beg,data->end());
            // normally 0.5 but we shift all equal points to the right (so this changes!)
            m_quality = m_qualityEval.compute(m_splitRatio,m_pointRatio,m_extentRatio);

            // to avoid this check if in tolerance (additional points < 5%) if not choose other axis and try again!
            //            bool notInTolerance = (data->size()/2 - std::distance(beg,splitRightIt))  >= 0.05 * data->size();

            //            if( notInTolerance ) {
            //                if(tries < NodeDataType::Dimension) {
            //                    ++tries;
            //                    m_splitAxis = (m_splitAxis + 1) % NodeDataType::Dimension;
            //                    return computeSplitPosition(splitRightIt,node,tries);
            //                }
            //            }

            break;
        }
        case Method::GEOMETRIC_MEAN: {
            m_splitPosition = data->getGeometricMean(m_splitAxis);
            if(!checkPosition(aabb)) {
                return false;
            }
            // Compute bodyRatio/splitRatio and quality
            m_splitRatio = computeSplitRatio(aabb);
            m_extentRatio = computeExtentRatio(aabb);
            m_pointRatio = computePointRatio(data);
            m_quality = m_qualityEval.compute(m_splitRatio,m_pointRatio,m_extentRatio);
            break;
        }
        }
        return true;
    }

    inline bool checkPosition(AABB<Dimension> & aabb) {
        ASSERTMSG( m_splitPosition >= aabb.m_minPoint(m_splitAxis)
                && m_splitPosition <= aabb.m_maxPoint(m_splitAxis), " split position wrong")

        if( (m_splitPosition - aabb.m_minPoint(m_splitAxis)) <= m_minExtent ||
                (aabb.m_maxPoint(m_splitAxis) - m_splitPosition) <= m_minExtent  ) {
            return false;
        }
        return true;
    }

    inline void updateSolution() {
        if( m_quality > m_bestQuality ) {
            m_wasLastTry = true;

            m_bestMethod = m_method;
            m_bestQuality = m_quality;
            m_bestSplitRightIt = m_splitRightIt;
            m_bestSplitAxis = m_splitAxis;
            m_bestSplitPosition = m_splitPosition;

            m_bestSplitRatio = m_splitRatio;
            m_bestPointRatio = m_pointRatio;
            m_bestExtentRatio = m_extentRatio;
        }
    }


    inline PREC computePointRatio(NodeDataType * data) {
        PREC n = 0.0;
        for(auto * p: *data) {
            if(Deref::get(p)(m_splitAxis) < m_splitPosition) {
                n+=1.0;
            }
        }
        n /= data->size();
        return (n>0.5)? 1.0-n : n ;
    }

    inline PREC computeSplitRatio(AABB<Dimension> & aabb) {
        PREC n = (m_splitPosition - aabb.m_minPoint(m_splitAxis))
                / (aabb.m_maxPoint(m_splitAxis) - aabb.m_minPoint(m_splitAxis)) ;
        ASSERTMSG(n>0.0 && n <=1.0," split ratio negative!, somthing wrong with splitPosition: " << n );
        return (n>0.5)? 1.0-n : n ;
    }

    inline PREC computeExtentRatio(AABB<Dimension> & aabb) {
        // take the lowest min/max extent ratio
        static ArrayStat<Dimension> t;
        t = m_extent;
        PREC tt = t(m_splitAxis);

        t(m_splitAxis) = m_splitPosition - aabb.m_minPoint(m_splitAxis);
        PREC r = t.minCoeff() / t.maxCoeff(); // gives NaN if max == 0, cannot happen since initial aabb is feasible!

        t(m_splitAxis) = tt; //restore
        t(m_splitAxis) = aabb.m_maxPoint(m_splitAxis) - m_splitPosition;
        r = std::min(r,t.minCoeff() / t.maxCoeff());

        ASSERTMSG(r > 0, "extent ratio <= 0!" );
        return r;

    }



    inline void computeStatistics() {
        ++m_splits;
        m_avgSplitRatio         += m_bestSplitRatio;
        m_avgPointRatio         += m_bestPointRatio;
        m_avgExtentRatio  += m_bestExtentRatio;
    }

    /** Statistics */
    unsigned int m_splitCalls = 0;
    unsigned int m_tries = 0;
    unsigned int m_splits = 0;
    PREC m_avgSplitRatio = 0;
    PREC m_avgPointRatio = 0;
    PREC m_avgExtentRatio = 0;


    QualityEvaluator m_qualityEval;

    /** Temp. values */
    ArrayStat<Dimension> m_extent; // current extent of the aabb
    iterator m_splitRightIt;
    PREC m_quality = 0.0;

    PREC m_splitRatio = 0; ///< ratio of the splitting plane in range (0,0.5)
    PREC m_pointRatio = 0; ///< ratio of points in left and right splittet box, in range [0,0.5]
    PREC m_extentRatio =  0; ///< minimal ratio of min/max extent of the left and right splitted box, in range (0,1] ;

    /** Min values to allow a split*/
    PREC m_minSplitRatio = 0.0;
    PREC m_minPointRatio = 0.0;
    PREC m_minExtentRatio = 0.0;

    Method m_method;
    PREC m_splitPosition = 0.0;
    SplitAxisType m_splitAxis;


    /** Best values */
    bool m_wasLastTry = false; ///< Flag which tells if the best values have just been updated when we finished the opt. loop
    bool m_found = false;
    iterator m_bestSplitRightIt;
    PREC m_bestQuality = std::numeric_limits<PREC>::lowest();
    PREC m_bestSplitRatio = 0, m_bestPointRatio = 0, m_bestExtentRatio = 0;
    Method m_bestMethod;
    PREC m_bestSplitPosition = 0.0;
    SplitAxisType m_bestSplitAxis;

    /** Fixed values */
    std::vector<SplitAxisType> m_splitAxes;
    std::vector<Method> m_methods;
    SearchCriteria m_searchCriteria;

    std::size_t m_allowSplitAboveMinPoints = 100;
    PREC m_minExtent = 0.0;

};

template<typename Traits> class TreeSimple;
template<typename Traits> class Tree;

/** So far this class is not working for N != 3, since the AABB<Dimension> class needs to be high-dimensional too!
TODO
*/
template<typename Traits>
class TreeNode {
private:
    friend class TreeSimple<Traits>;
    friend class Tree<Traits>;
public:
    DEFINE_KDTREE_TYPES( Traits )
    DEFINE_LAYOUT_CONFIG_TYPES

    using  SplitAxisType = char;
    STATIC_ASSERT( Dimension <= std::numeric_limits<char>::max());

    struct BoundaryInformation {
        TreeNode* m_nodes[Dimension][2] = {{nullptr}}; ///< min/max pointers , if nullptr its an outside boundary
    };

    TreeNode(
            std::size_t idx,
            AABB<Dimension> aabb,
            NodeDataType * data,
            unsigned int treeLevel = 0,
            SplitAxisType splitAxis = -1, /* always a leaf */
            PREC splitPosition = 0.0)
        : m_idx(idx), m_aabb(aabb),m_splitAxis(splitAxis), m_treeLevel(treeLevel),
          m_splitPosition(splitPosition), m_data(data), m_bound(new BoundaryInformation{}) {
    }

    ~TreeNode() {
        cleanUp();
    };

    void setBoundaryInfo(const BoundaryInformation & b) {
        if(!m_bound) {
            m_bound = new BoundaryInformation(b);
        }
        *m_bound = b;
    }


    inline TreeNode * leftNode() {
        return m_child[0];
    }
    inline const TreeNode * leftNode() const {
        return m_child[0];
    }

    inline TreeNode * rightNode() {
        return m_child[1];
    }
    inline const TreeNode * rightNode()const {
        return m_child[1];
    }


    inline NodeDataType * data() {
        return m_data;
    };
    inline const NodeDataType * data() const {
        return m_data;
    };

    /** Splits the node into two new nodes by the splitting position*/
    bool split(SplitHeuristicType & s) {

        auto pLR = s.doSplit(this);

        if(pLR.first == nullptr) { // split has failed!
            return false;
        }


        // Split aabb and make left and right
        // left (idx for next child if tree is complete binary tree, then idx = 2*idx+1 for left child)
        AABB<Dimension> t(m_aabb);
        PREC v = t.m_maxPoint(m_splitAxis);
        t.m_maxPoint(m_splitAxis) = m_splitPosition;
        m_child[0] = new TreeNode(2*m_idx+1,t,pLR.first,  m_treeLevel+1);

        // right
        t.m_maxPoint(m_splitAxis) = v; //restore
        t.m_minPoint(m_splitAxis) = m_splitPosition;
        m_child[1] = new TreeNode(2*m_idx+2,t,pLR.second, m_treeLevel+1);

        // Set Boundary Information
        BoundaryInformation b = *m_bound;
        TreeNode * tn = b.m_nodes[m_splitAxis][1];
        b.m_nodes[m_splitAxis][1] = m_child[1]; // left changes pointer at max value
        m_child[0]->setBoundaryInfo(b);

        b.m_nodes[m_splitAxis][1] = tn; // restore
        b.m_nodes[m_splitAxis][0] = m_child[0]; // right changes pointer at min value
        m_child[1]->setBoundaryInfo(b);

        // clean up own node:
        if(m_treeLevel != 0) {
            cleanUp();
        }


        return true;
    }

    template<typename NeighbourIdxMap>
    void getNeighbourLeafsIdx( NeighbourIdxMap & neigbourIdx, PREC minExtent) {
        // we determine in each direction of the kd-Tree in R^n (if 3D, 3 directions)
        // for "min" and "max" the corresponding leafs in the subtrees given by
        // the boundary information in the leaf node:
        // e.g. for the "max" direction in x for one leaf, we traverse the  subtree of the boundary
        // information in "max" x direction
        // for "max" we always take the left node  (is the one which is closer to our max x boundary)
        // for "min" we always take the right node (is the one which is closer to our min x boundary)
        // in this way we get all candidate leaf nodes (which share the same split axis with split position s)
        // which need still to be checked if they are neighbours
        // this is done by checking if the boundary subspace with the corresponding axis set to the split position s
        // (space without the axis which is checked, e.g y-z space, with x = s)
        // against the current leaf nodes boundary subspace
        // (which is thickened up by the amount of the minimal leaf node extent size,
        // important because its not clear what should count as a neighbour or what not)
        // if this neighbour n subspace overlaps the thickened leaf node subspace then this node is
        // considered as a neighbour for leaf l, (and also neighbour n has l as neighbour obviously)
        // the tree should be build with a slightly bigger min_extent size than the thickening in the step here!
        // to avoid to many nodes to be classified as neighbours (trivial example, min_extent grid)
        // if we have no boundary information


        ASSERTMSG(m_bound, "To determine neighbours we need boundary information!")

        std::deque<TreeNode*> nodes; // Breath First Search
        auto & neighbours = neigbourIdx[m_idx]; // Get this neighbour map
        TreeNode * f;

        AABB<Dimension> aabb(m_aabb);
        aabb.expand(minExtent); // expand this nodes aabb such that we find all the neighbours which overlap this aabb;

        // For each axis traverse subtree
        for(SplitAxisType d = 0; d<Dimension; ++d) {

            // for min and max
            for(unsigned int m = 0; m<2; ++m) {

                // push first -> Breath First Search (of boundary subtree)
                TreeNode * subTreeRoot = m_bound->m_nodes[d][m];
                if(!subTreeRoot) {
                    continue;
                } else {
                    nodes.emplace_back(subTreeRoot);
                }

                while(!nodes.empty()) {
                    f = nodes.front();
                    auto axis = f->m_splitAxis;
                    if(f->isLeaf()) {
                        //std::cout << "is leaf" << std::endl;
                        // determine if f is a neighbour to this node
                        // if leaf is not already in neighbour map for this node (skip)
                        if( neighbours.find(f->m_idx) == neighbours.end() ) {

                            // check if the subspace (fixedAxis = m) overlaps
                            if(aabb.overlapsSubSpace(f->m_aabb,d)) {
                                //std::cout << m_idx << " overlaps" << f->getIdx() <<  std::endl;
                                neighbours.emplace(f->m_idx);
                                neigbourIdx[f->m_idx].emplace(m_idx);
                            }

                        }

                    } else {
                        // if the node f currently processed has the same split axis as the boundary direction
                        // add only the nodes closer to the cell of this node
                        if(axis == d) {
                            if(m==0) {
                                // for minmal boundary only visit right nodes (closer to this)
                                nodes.emplace_back(f->rightNode());
                            } else {
                                // for maximal boundary only visit left nodes (closer to this)
                                nodes.emplace_back(f->leftNode());
                            }
                        } else {
                            // otherwise add all
                            nodes.emplace_back(f->leftNode());
                            nodes.emplace_back(f->rightNode());
                        }
                    }
                    nodes.pop_front();
                }


            }

        }

    }

    inline AABB<Dimension> & aabb() {
        return m_aabb;
    }
    inline const AABB<Dimension> & aabb() const {
        return m_aabb;
    }

    inline bool hasLeftChildren() const{
        return m_child[0];
    }
    inline bool hasChildren() const{
        return m_child[0] && m_child[1];
    }

    inline bool isLeaf() const{
        return (m_splitAxis < 0);
    }

//    inline void setIdx(std::size_t idx) {
//        m_idx=idx;
//    }
    inline std::size_t getIdx() const {
        return m_idx;
    }

    inline void setSplitAxis(SplitAxisType splitAxis) {
        m_splitAxis= splitAxis ;
    }
    inline void setSplitPosition(PREC splitPos) {
        m_splitPosition= splitPos ;
    }

    inline SplitAxisType getSplitAxis() const{
        return m_splitAxis ;
    }
    inline PREC getSplitPosition() const{
        return m_splitPosition;
    }
    inline PREC getSplitRatio() const{
        return (m_splitPosition - m_aabb.m_minPoint(m_splitAxis) )
                / (m_aabb.m_maxPoint(m_splitAxis)-m_aabb.m_minPoint(m_splitAxis));
    }

    inline unsigned int getLevel() const{
        return m_treeLevel;
    }

    void cleanUp(bool data = true, bool bounds = true) {
        if(data && m_data) {
            delete m_data;
            m_data = nullptr;
        }
        if(bounds && m_bound) {
            delete m_bound;
            m_bound = nullptr;
        }
    }
    template<typename T>
    void writeXML(T * rootNode) {

    }

private:
    std::size_t m_idx = std::numeric_limits<std::size_t>::max();   ///< leaf node index (only sensfull for leaf nodes)
    unsigned int m_treeLevel = 0; ///< root level is 0
    AABB<Dimension> m_aabb;

    /** Boundary information which is nullptr for non-leaf nodes */
    BoundaryInformation * m_bound = nullptr;

    NodeDataType* m_data = nullptr;

    SplitAxisType m_splitAxis = -1; ///< smaller than zero to indicate leaf node!
    PREC m_splitPosition = 0.0;
    /** Child Nodes */
    TreeNode * m_child[2] = {nullptr ,nullptr};
};



template<typename TNodeData = PointData<>,
        template<typename...> class TSplitHeuristic = meta::bind_front< meta::quote<SplitHeuristicPointData>, LinearQualityEvaluator>::template apply
        >
struct TreeTraits {

    using NodeDataType = TNodeData;
    static const unsigned int Dimension = NodeDataType::Dimension;
    using NodeType = TreeNode<TreeTraits>;
    using SplitHeuristicType = TSplitHeuristic<TreeTraits>;
};


template<typename Traits = TreeTraits<> >
class TreeSimple{
public:

    DEFINE_KDTREE_TYPES( Traits )

    TreeSimple(){}

    ~TreeSimple(){
       resetTree();
    }
    /** Built a tree from a node map and links
    * \p c   is a associative container of nodes with type \tp NodeType where the key type is std::size_t and
    * value type is a pointe to type NodeType. The tree owns the pointers afterwards!
    * \p links is an associative container with type \tp NodeToChildMap
    * where the key is std::size_t and specifies the parent and the value type is a std::pair<std::size_t,std::size_t>
    * for left and right child node indices in the map \p c.
    */
    template<typename NodeMap, typename NodeToChildMap>
    void build( NodeType * root, NodeMap & c , NodeToChildMap & links){

            if( c.find(root->getIdx()) == c.end()){
                ERRORMSG("Root node not in NodeMap!")
            }

            resetTree();

            std::unordered_set<std::size_t> hasParent;
            // first link all nodes together
            auto itE = c.end();
            for(auto & l : links){ // first idx, second pair<idxL,idxR>
                auto it  = c.find(l.first);
                auto itL = c.find(l.second.first);
                auto itR = c.find(l.second.second);

                if(it==itE || itL==itE || itR==itE){
                    ERRORMSG("Link at node idx: " << l.first << " wrong!")
                }

                if(!hasParent.emplace(l.second.first).second){
                    ERRORMSG("Node idx: " << l.second.first << "has already a parent!")
                };
                if(!hasParent.emplace(l.second.second).second){
                    ERRORMSG("Node idx: " << l.second.first << "has already a parent!")
                };
                if( !it->second || !itL->second || !itR->second){
                    ERRORMSG("Ptr for link zero")
                }
                it->second->m_child[0] = itL->second; // link left
                it->second->m_child[1] = itR->second; // link right
            }

            if(hasParent.size() != c.size()-1){
                ERRORMSG("Tree needs to have N nodes, with one root, which gives N-1 parents!")
            }

            // Save root as it is a valid binary tree
            m_root = root;
    }

    void resetTree(){
        if(m_root){
            deleteRecursively(m_root);
            m_root = nullptr;
        }
    }

    /** Get cell index of the leaf which owns point \p point
    * \p point is the d-dimensional point in the frame of reference the kd-Tree was built!
    * Points outside the roots AABB box, are naturally project to the most outer leaf automatically.
    */
    template<typename Derived>
    const NodeType * getLeaf(const MatrixBase<Derived> & point) const{
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived,Dimension);
        // Recursively traverse tree to find the leaf which contains the point
        ASSERTMSG(m_root, "Tree is not built!")
        const NodeType * currentNode = m_root;

        while(!currentNode->isLeaf()){
            // all points greater or equal to the splitPosition belong to the right node
            if(point(currentNode->getSplitAxis()) >= currentNode->getSplitPosition()){
                currentNode = currentNode->rightNode();
            }else{
                currentNode = currentNode->leftNode();
            }
        }
        return currentNode;
    }

protected:

    void deleteRecursively(const NodeType* n){
        if(n){
            auto * l = n->leftNode();
            auto * r = n->rightNode();
            if(l){ deleteRecursively(l); }
            if(r){ deleteRecursively(r); }
        }
        delete n;
    }

    NodeType * m_root = nullptr;     ///< Root node
};

/** Standart Class to build a kd-tree */
template<typename Traits = TreeTraits<> >
class Tree : protected TreeSimple<Traits> {

public:
    using Base = TreeSimple<Traits>;
    DEFINE_KDTREE_TYPES( Traits )

    Tree() {}
    ~Tree() {
        resetTree();
    }

    void resetTree(){
        m_leafs.clear();
        resetStatistics();

        for(auto * n: m_nodes){
            delete n;
        }
        m_nodes.clear();
        this->m_root = nullptr;

        Base::resetTree();
    }

    /** Builds a new Tree with the SplitHeurstic */
    template<bool computeStatistics = true>
    void build(const AABB<Dimension> & aabb, std::unique_ptr<NodeDataType> data,
            unsigned int maxTreeDepth = 50,
            unsigned int maxLeafs = std::numeric_limits<unsigned int>::max()) {

        resetTree();
        m_heuristic.resetStatistics();

        m_maxTreeDepth = maxTreeDepth;
        m_maxLeafs = maxLeafs;


        if((aabb.extent() <= 0.0).any()) {
            ERRORMSG("AABB given has wrong extent!");
        }
        this->m_root = new NodeType(0,aabb,data.release());

        std::deque<NodeType*> splitList; // Breath first splitting
        splitList.push_back(this->m_root);
        m_nodes.push_back(this->m_root);

        bool nodeSplitted;

        auto end = splitList.end();
        auto it  = splitList.begin();

        m_treeDepth = 0;
        unsigned int nNodesLevelCurr = 1; // number of nodes in list of current level
        unsigned int nNodesLevelNext = 0; // number of nodes in list (after the current nodes with next level)

        unsigned int nLeafs = 1;
//        unsigned int nodeIdx = 0; // root node has idx = 0;

        auto greaterData = [](const NodeType * a, const NodeType * b) {
            return a->data()->size() > b->data()->size() ;
        };

        while( !splitList.empty()) {


            auto * f = splitList.front();

            // first check if number of nodes at curr level is zero
            if(nNodesLevelCurr==0) {
                std::cout << "Tree Level: " << m_treeDepth << " done, added childs: " << nNodesLevelNext << std::endl;
                ++m_treeDepth;

                std::swap(nNodesLevelCurr, nNodesLevelNext);

                // may be sort the child nodes in splitList according to their data size
                // (slow for big trees) std::sort(splitList.begin(),splitList.end(),greaterData);

                f = splitList.front();
                //std::cout << "biggest leaf size: " << splitList.front()->data()->size() << std::endl;
            }

            if(m_treeDepth+1 <= m_maxTreeDepth && nLeafs < m_maxLeafs) {

                // try to split the nodes in the  list
                nodeSplitted = f->split(m_heuristic);

                if(nodeSplitted) {
                    auto * l = f->leftNode();
                    auto * r = f->rightNode();
                    splitList.emplace_back(l); // add to front
                    splitList.emplace_back(r);// add to front

                    // Push to total list
                    m_nodes.emplace_back(l);
                    m_nodes.emplace_back(r);

                    nNodesLevelNext+=2;
                    ++nLeafs; // each split adds one leaf

                } else {
                    // this is a leaf node, save in leaf list (later in enumerateNodes):
                    //std::cout << "leaf size: " << f->data()->size() << ",";
                }

                // add to node statistic:
                if(computeStatistics) {
                    addToNodeStatistics(f);
                }

            } else {
                // depth level reached
                // add to node statistics
                if(computeStatistics) {
                    addToNodeStatistics(f);
                }

            }
            --nNodesLevelCurr;
            // pop node at the front
            splitList.pop_front();
        }

        // enumerate nodes new (firsts leafs then all other nodes)
        enumerateNodes();

        if(computeStatistics) {
            averageStatistics();
        }
    }


    /** Deletes the data and bounds in each node if specified */
    void cleanUp(bool data = true, bool bounds = true) {
        if(!data && !bounds) {
            return;
        }
        for(auto * p : m_nodes) {
            p->cleanUp(data,bounds);
        }
    }

    template<typename... T>
    void initSplitHeuristic(T &&... t) {
        m_heuristic.init(std::forward<T>(t)...);
    }

    template<bool safetyCheck = true>
    std::unordered_map<std::size_t, std::unordered_set<std::size_t> >
    buildLeafNeighbours(PREC minExtent) {
        if(!this->m_root) {
            ERRORMSG("There is not root node! KdTree not built!")
        }

        // Get all leaf neighbour indices for each leaf node!
        // To do this, we traverse the leaf list and for each leaf l
        // we determine in each direction of the kd-Tree in R^n (if 3D, 3 directions)
        // for "min" and "max" the corresponding leafs in the subtrees given by
        // the boundary information in the leaf node:
        // e.g. for the "max" direction in x for one leaf, we traverse the  subtree of the boundary
        // information in "max" x direction
        // for "max" we always take the left node  (is the one which is closer to our max x boundary)
        // for "min" we always take the right node (is the one which is closer to our min x boundary)
        // in this way we get all candidate leaf nodes (which share the same split axis with split position s)
        // which need still to be checked if they are neighbours
        // this is done by checking if the boundary subspace with the corresponding axis set to the split position s
        // (space without the axis which is checked, e.g y-z space, with x = s)
        // against the current leaf nodes boundary subspace
        // (which is thickened up by the amount of the minimal leaf node extent size,
        // important because its not clear what should count as a neighbout or what not)
        // if this neighbour n subspace overlaps the thickened leaf node subspace then this node is
        // considered as a neighbour for leaf l, (and also neighbour n has l as neighbour obviously)
        // If the tree is build with the same min_extent size as the thickening in this step here, then it should work
        // since the tree has all extents striclty greater then min_extent, and here
        // to avoid to many nodes to be classified as neighbours (trivial example, min_extent grid)

        // each leaf gets a
        std::unordered_map<std::size_t, std::unordered_set<std::size_t> > leafToNeighbourIdx;

        // iterate over all leafs
        for(auto & p: m_leafs) {
            p.second->getNeighbourLeafsIdx(leafToNeighbourIdx, minExtent);
        }

        // Do safety check in debug mode
        if(safetyCheck){
            safetyCheckNeighbours(leafToNeighbourIdx,minExtent);
        }


        return leafToNeighbourIdx;
    }


    /** Returns tuple with values
    * (number of leafs, avg. leaf data size, min. leaf data size, max. leaf data size)
    */
    std::tuple<unsigned int, std::size_t, std::size_t, std::size_t, std::size_t, std::size_t, PREC, PREC>
    getStatistics() {
        return std::make_tuple(m_treeDepth,
                m_nodes.size(),
                m_leafs.size(),
                m_avgLeafSize,
                m_minLeafDataSize,
                m_maxLeafDataSize,
                m_minLeafExtent,
                m_maxLeafExtent);
    }

    std::string getStatisticsString() {
        std::stringstream s;
        auto t = getStatistics();
        std::string h = m_heuristic.getStatisticsString();
        s << "Tree Stats: "
                << "\n\t tree level : " << std::get<0>(t)
                << "\n\t nodes      : " << std::get<1>(t)
                << "\n\t leafs      : " << std::get<2>(t)
                << "\n\t avg. leaf data size : " << std::get<3>(t)
                << "\n\t min. leaf data size : " << std::get<4>(t)
                << "\n\t max. leaf data size : " << std::get<5>(t)
                << "\n\t min. leaf extent    : " << std::get<6>(t)
                << "\n\t max. leaf extent    : " << std::get<7>(t)
                << "\nSplitHeuristics Stats: \n"
                << h << std::endl;

        return s.str();
    }

    void saveToXML(const boost::filesystem::path & folder = "./") {
        std::stringstream ss;

        if(!this->m_root) {
            ERRORMSG("No root node in kdTree!")
        }

        boost::filesystem::path filePath = folder;

        std::string filename = "TopologyInfo_1";
        filePath /= filename + ".xml";

        // Open XML and write structure!
        pugi::xml_document dataXML;
        std::stringstream xml("<TopologyBuilder type=\"KdTree\" buildMode=\"\" >"
                "<Description>"
                "A_IK is tranformation matrix, which transforms points from frame K to frame I\n"
                "AABBList contains all AABBs from all ranks in frame I\n"
                "Time is the current simulation time\n"
                "BuiltTime is the elapsed time to build the topology\n"
                "AABBTree contains the tree without the leafs\n"
                "</Description>"
                "<Time value=\"0\" />"
                "<BuiltTime value=\"0\" />"
                "<AABBList />"
                "<KdTree aligned=\"\">"
                "<Root/>"
                "<Leafs/>"
                "<AABBTree/>"
                "<A_IK/>"
                "</KdTree>"
                "<Points/>"
                "</TopologyBuilder>");

        bool res = dataXML.load(xml);
        ASSERTMSG(res,"Could not load initial xml data file");

        // Write data

        using XMLNodeType = pugi::xml_node;
        XMLNodeType node;
        static const auto  nodePCData = pugi::node_pcdata;
        XMLNodeType root =  dataXML.child("TopologyBuilder");

        XMLNodeType kdTreeNode = root.first_element_by_path("./KdTree");

        kdTreeNode.attribute("aligned").set_value( true );

        XMLNodeType r = kdTreeNode.child("Root");
        XMLNodeType aabb = r.append_child("AABB");
        ss.str("");
        ss << Utilities::typeToString(this->m_root->aabb().m_minPoint.transpose().format(MyMatrixIOFormat::SpaceSep)).c_str() <<" "
                << Utilities::typeToString(this->m_root->aabb().m_maxPoint.transpose().format(MyMatrixIOFormat::SpaceSep)).c_str() << "\n";
        aabb.append_child(nodePCData).set_value(ss.str().c_str());

        // Save leafs
        XMLNodeType leafs = kdTreeNode.child("Leafs");
        unsigned int level = 0;
        for(auto & p: m_leafs) {
            auto * l = p.second;
            XMLNodeType node = leafs.append_child("Leaf");
            node.append_attribute("level").set_value(l->getLevel());
            node.append_attribute("idx").set_value(std::to_string(l->getIdx()).c_str());
            aabb = node.append_child("AABB");
            ss.str("");
            ss << Utilities::typeToString(l->aabb().m_minPoint.transpose().format(MyMatrixIOFormat::SpaceSep)).c_str() <<" "
                    << Utilities::typeToString(l->aabb().m_maxPoint.transpose().format(MyMatrixIOFormat::SpaceSep)).c_str() << "\n";
            aabb.append_child(nodePCData).set_value(ss.str().c_str());

            node = node.append_child("Points");
//            ss.str("");
//            for(auto * p : *(l->data())){
//                ss << p->transpose().format(MyMatrixIOFormat::SpaceSep) << std::endl;
//            }
            //node.append_child(nodePCData).set_value( ss.str().c_str() );
        }

        // Save AABB tree (breath first)
        XMLNodeType aabbTree = kdTreeNode.child("AABBTree");
        std::deque<NodeType*> q; // Breath first queue

        q.push_back(this->m_root);
        unsigned int currLevel = this->m_root->getLevel();
        ss.str("");
        while(q.size()>0) {
            // Write stuff of f if not leaf
            auto * f = q.front();

            if(f->getLevel() > currLevel) {
                // write new string
                aabb = aabbTree.append_child("AABBSubTree");
                aabb.append_attribute("level").set_value(currLevel);
                aabb.append_child(nodePCData).set_value( ss.str().c_str() );
                // update to next level
                currLevel = f->getLevel();
                ss.str("");
            }

            if(!f->isLeaf()) {
                ss << Utilities::typeToString(f->aabb().m_minPoint.transpose().format(MyMatrixIOFormat::SpaceSep)).c_str() <<" "
                        << Utilities::typeToString(f->aabb().m_maxPoint.transpose().format(MyMatrixIOFormat::SpaceSep)).c_str() << "\n";
            }

            // push the left/right
            auto * n = f->leftNode();
            if(n) {
                q.push_back(n);
            }
            n = f->rightNode();
            if(n) {
                q.push_back(n);
            }

            q.pop_front();
        }

        // write last string
        auto s = ss.str();
        if(!s.empty()) {
            aabb = aabbTree.append_child("AABBSubTree");
            aabb.append_attribute("level").set_value(currLevel);
            aabb.append_child(nodePCData).set_value( s.c_str() );
        }


        // Write A_IK (as quaternion)
        node = kdTreeNode.child("A_IK");
        node.append_child(nodePCData).set_value( Utilities::typeToString(Matrix33::Identity().format(MyMatrixIOFormat::SpaceSep)).c_str() );

        //Write all points if data is available
//        if(m_root->data()){
//            auto node = root.child("Points");
//            node.append_child(nodePCData).set_value( m_root->data()->getPointString().c_str() );
//        }

        dataXML.save_file(filePath.c_str(),"    ");
    }

    template<typename Derived>
    inline const NodeType * getLeaf(const MatrixBase<Derived> & point) const{
        return Base::getLeaf(point);
    }

    const NodeType * getLeaf(const std::size_t & index) const{
        auto it = m_leafs.find(index);
        if(it == m_leafs.end()){
            return nullptr;
        }
        return it->second;
    }

private:


    SplitHeuristicType m_heuristic;

    std::unordered_map<std::size_t, NodeType *> m_leafs; ///< Only leaf nodes (idx to node)
    std::vector<NodeType *> m_nodes; ///< All nodes

    unsigned int m_maxTreeDepth = 50;
    unsigned int m_maxLeafs = std::numeric_limits<unsigned int>::max();

    /** Statistics ========================*/
    unsigned int m_treeDepth;
    PREC m_avgSplitPercentage;
    /** Min/Max Extent for Leafs */
    PREC m_minLeafExtent;
    PREC m_maxLeafExtent;
    /** Data Sizes for Leafs*/
    std::size_t m_avgLeafSize;
    std::size_t m_minLeafDataSize;
    std::size_t m_maxLeafDataSize;


    void addToNodeStatistics(NodeType * n) {
        if(n->isLeaf()) {
            auto s = n->data()->size();
            m_avgLeafSize += s;
            m_minLeafDataSize = std::min(m_minLeafDataSize,s);
            m_maxLeafDataSize = std::max(m_maxLeafDataSize,s);
            m_minLeafExtent = std::min(m_minLeafExtent,n->aabb().extent().minCoeff());
            m_maxLeafExtent = std::max(m_maxLeafExtent,n->aabb().extent().maxCoeff());
        }
    }

    void averageStatistics() {
        m_avgLeafSize /= m_leafs.size();
        m_avgSplitPercentage /= m_nodes.size() - m_leafs.size();
    }

    void resetStatistics() {
        m_treeDepth = 0;
        m_avgSplitPercentage = 0.0;
        m_minLeafExtent = std::numeric_limits<PREC>::max();
        m_maxLeafExtent = 0.0;
        m_avgLeafSize = 0;
        m_minLeafDataSize = std::numeric_limits<std::size_t>::max();
        m_maxLeafDataSize = 0;
    }

    /** Enumerate nodes (continously, leafs first, then non-leafs */
    void enumerateNodes(){

        std::size_t leafIdx = 0;

        for(auto * n : m_nodes){
            if(n->isLeaf()){
                n->m_idx=leafIdx++;
                m_leafs.emplace(n->m_idx,n);
            }
        }
        std::size_t nonleafIdx = m_leafs.size();

        for(auto * n : m_nodes){
            if(!n->isLeaf()){
                n->m_idx = nonleafIdx++;
            }
        }
    }

    /** Safety check for neighbour list */
    template<typename NeighbourMap>
    void safetyCheckNeighbours(const NeighbourMap & n, PREC minExtent) {

        if(n.size() != m_leafs.size()){
            ERRORMSG("Safety check for neighbours failed!: size")
        }

        bool ok = true;
        for(auto & p:  m_leafs){
            auto * l = p.second;
            // Check leaf l
            AABB<Dimension> t = l->aabb();
            t.expand(minExtent);
            // check against neighbours
            auto it = n.find(p.first);
            if(it == n.end()){
                ERRORMSG("Safety check for neighbours failed!: find")
            }

            for(const auto & idx : it->second ){
                if(m_leafs.find(idx) == m_leafs.end()){
                    ERRORMSG("Safety check for neighbours failed!: find idx")
                }
                ok &= t.overlaps( m_leafs[idx]->aabb() );
            }
        }
        if(!ok){
            ERRORMSG("Safety check for neighbours failed!: ok")
        }
    }
};

};


#endif
