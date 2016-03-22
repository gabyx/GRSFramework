    #ifndef ConvexHull2D_hpp
    #define ConvexHull2D_hpp

    #include <vector>

    #include "AssertionDebug.hpp"

    #include "TypeDefs.hpp"
    #include "TypeDefsPoints.hpp"

    #include "PointFunctions.hpp"
    #include "ContainerFunctions.hpp"

    /** Generate convex hull of 2D points
    * Side Effects: Removes equal points from input point list reference, input points are angle sorted afterwards
    * Function getIndices() returns the ascending indices of the sorted point list which span the convex hull.
    */
    class ConvexHull2D {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        DEFINE_MATRIX_TYPES
        DEFINE_POINTS_CONFIG_TYPES

        /** Cosntructor, points is not a temporary, it accepts all sorts of matrix expressions,
        * however the construction of MatrixRef<> might create a temporary but this is stored in m_p!
        */
        template<typename Derived>
        ConvexHull2D(const MatrixBase<Derived> & points) :m_p(points) {
            EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived,2, Eigen::Dynamic)
            ASSERTMSG( m_p.data() == points.derived().data() ," You store a temporary in a Ref<> which works here, but do you really want this?")
        }

        void compute() {
            using namespace PointFunctions;
            using namespace ContainerFunctions;

            m_indicesCH.clear();

            // Need at least 1 point! m_p.col(0)
            if(m_p.cols()==0) {
                return;
            }

            // Compute min point p0
            unsigned int position = minPointYX(m_p);

            //Save p0 as first point in indices list
            std::vector<unsigned int> indices; // all point indices to test!
            indices.push_back(position);

            //Add all indices of points not almostEqual to minIt
            Vector2 base = m_p.col(position);
            for(unsigned int i = 0; i<m_p.cols(); ++i) {
                if( (i != position) && !almostEqual(base,m_p.col(i)) ) {
                    indices.push_back(i);
                }
            }

            // Utilities::printVector(std::cout,indices.begin(),indices.end(),",");
            // std::cout << std::endl;

            // Convex hull consists of only 1 or 2 points!
            if(indices.size() <= 2  ) {
                m_indicesCH.assign(indices.begin(),indices.end());
                return;
            }

            // Sort by angle
            CompareByAngle comp(m_p,base);
            std::sort( indices.begin() + 1, indices.end(), comp );
            //Utilities::printVector(std::cout,indices.begin(),indices.end(),",");
            //std::cout << std::endl;

            // and remove consceutive almost equal elements
            auto d2 = moveConsecutiveToBackIf( indices.begin() + 1,indices.end(),
            [&](unsigned int a, unsigned int b) {
                return PointFunctions::almostEqual(this->m_p.col(a),this->m_p.col(b));
            });
            indices.resize( std::distance(indices.begin(),d2) );

            // Convex hull consists of only 1 or 2 points!
            if(indices.size() <= 2  ) {
                m_indicesCH.assign(indices.begin(),indices.end());
                return;
            }

            // Perform Graham Scan over indices
            // Iterate over point list starting at 1:
            // Start with triangle p0,p1,p2

            m_indicesCH.push_back( indices[0] );
            m_indicesCH.push_back( indices[1] );
            unsigned int i = 2; // runs with indices
            unsigned int lPtIdx = indices[0] ;
            unsigned int mPtIdx= indices[1];
            while( i<indices.size()) {

                unsigned int currIdx = indices[i];

                if( almostEqual( m_p.col( currIdx ), m_p.col(mPtIdx) ) ) {
                    ++i;//skip point
                    continue;
                }

                ASSERTMSG( lPtIdx < m_p.size() && mPtIdx < m_p.size() && currIdx < m_p.size(), "?")

                if( areaSign( m_p.col(lPtIdx), m_p.col(mPtIdx), m_p.col(currIdx)) > 0) { // if we have a left turn!
                    //std::cout << "Left turn: c:" << currIdx << " m:" << mPtIdx << " l: "<< lPtIdx << " sign: " << areaSign( m_p.col(lPtIdx), m_p.col(mPtIdx), m_p.col(currIdx)) << std::endl;

                    m_indicesCH.push_back(currIdx); //add currIdx to convex hull list
                    lPtIdx = mPtIdx;          // last becomes middle
                    mPtIdx = currIdx;         // middle becomes currIdx
                    ++i;                      // next point

                } else { // if we have a right turn
                    //std::cout << "Right turn: c:" << currIdx << " m:" << mPtIdx << " l: "<< lPtIdx << " sign: " << areaSign( m_p.col(lPtIdx), m_p.col(mPtIdx), m_p.col(currIdx)) << std::endl;
                    m_indicesCH.pop_back();   // remove middle point;
                    // move triplet back if possible
                    if( m_indicesCH.size() <= 1) { // Degenerate Case if we come back to the begining
                        m_indicesCH.push_back(currIdx);
                        mPtIdx = currIdx;
                        ++i; // next point
                    } else {
                        // move triplet back
                        mPtIdx = lPtIdx;                 // middle point becomes last
                        lPtIdx = *(++m_indicesCH.rbegin());   // previous of .back();
                    }
                }

            }

            //std::cout << "indices End: " << m_indicesCH.size() << std::endl;
        }

        bool verifyHull() {
            using namespace PointFunctions;
            if(m_indicesCH.size()>2) {
                unsigned int i = 2;
                while( i<m_indicesCH.size()) {
                    if( ! (areaSign( m_p.col(m_indicesCH[i-2]), m_p.col(m_indicesCH[i-1]), m_p.col(m_indicesCH[i])) >= 0) ) { // if this is not a left turn
                        return false;
                    }
                    ++i;
                }
            }
            return true;
        }

        std::vector<unsigned int> & getIndices() {
            return m_indicesCH;
        }
    private:
        std::vector<unsigned int> m_indicesCH;
        const MatrixRef< const Matrix2Dyn > m_p;
    };

#endif
