// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef MVBB_hpp
#define MVBB_hpp


#include "TypeDefs.hpp"
#include "TypeDefsPoints.hpp"

#include "GreatestCommonDivisor.hpp"
#include "ProjectedPointSet.hpp"
#include "OOBB.hpp"

namespace ComputeMVBB {

    DEFINE_MATRIX_TYPES
    DEFINE_POINTS_CONFIG_TYPES

/** We are given a point set, and (hopefully) a tight fitting
*   bounding box. We compute a sample of the given size nPoints that represents
*   the point-set. The only guarenteed is that if we use sample of size m,
*   we get an approximation of quality about 1/\sqrt{m}. Note that we pad
*   the sample if necessary to get the desired size.
*   @param nPoints needs to be greater or equal than 2
*/
template<typename Derived>
void samplePointsGrid(Matrix3Dyn & newPoints,
                      const MatrixBase<Derived> & points,
                      const unsigned int nPoints,
                      OOBB oobb) {


    if(nPoints > points.cols() || nPoints < 2) {
        ERRORMSG("Wrong arguements!")
    }

    newPoints.resize(3,nPoints);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<unsigned int> dis(0, points.cols()-1);

    //total points = bottomPoints=gridSize^2  + topPoints=gridSize^2
    unsigned int gridSize = std::min( static_cast<unsigned int>( std::sqrt( static_cast<double>(nPoints) / 2.0 )) , 1U );

    // Set z-Axis to longest dimension
    oobb.setZAxisLongest();

    Vector3 dirZ = oobb.getDirection(2); // in I Frame

    unsigned int halfSampleSize = gridSize*gridSize;
    std::vector< unsigned int > topPoints(halfSampleSize, 0);    // grid of indices of the top points (1-indexed)
    std::vector< unsigned int > bottomPoints(halfSampleSize, 0); // grid of indices of the bottom points (1-indexed)

    using LongInt = long long int;
    MyMatrix<LongInt>::Array2 idx; // Normalized P
    Array2 dxdyInv =  Array2(gridSize,gridSize) / oobb.extent().head<2>(); // in K Frame;
    Vector3 K_p;

    Matrix33 A_KI(oobb.m_q_KI.matrix().transpose());

    // Register points in grid
    for(unsigned int i=0; i<nPoints; ++i) {

        K_p = A_KI * points.col(i);
        // get x index in grid
        idx = (  (K_p - oobb.m_minPoint).head<2>().array() * dxdyInv ).template cast<LongInt>();
        // map to grid
        idx(0) = std::max(   std::min( LongInt(gridSize-1), idx(0)),  0LL   );
        idx(1) = std::max(   std::min( LongInt(gridSize-1), idx(1)),  0LL   );

        // Register points in grid
        // if z axis of p is > topPoints[pos]  -> set new top point at pos
        // if z axis of p is < bottom[pos]     -> set new bottom point at pos
        unsigned int pos = idx(0) + idx(1)*gridSize;
        if( topPoints[pos] == 0) {
            topPoints[pos] = bottomPoints[pos] = i+1;
        } else {
            PREC Ztop    = points.col(topPoints[pos]).dot(dirZ);
            PREC Zbottom = points.col(bottomPoints[pos]).dot(dirZ);
            if( Ztop < K_p(2) ) {
                topPoints[pos] = i+1;
            }
            if( Zbottom > K_p(2) ) {
                bottomPoints[pos] = i+1;
            }
        }
    }

    // Copy top and bottom points
    unsigned int k=0;

    // k does not overflow -> 2* halfSampleSize = 2*gridSize*gridSize <= nPoints;
    ASSERTMSG(halfSampleSize<=nPoints,"?")
    for(unsigned int i=0; i<halfSampleSize; ++i) {
        if( topPoints[i] != 0 ) {
            //Array3 a(i % gridSize,i/gridSize,oobb.m_maxPoint(2)-oobb.m_minPoint(2));
            //a.head<2>()*=dxdyInv.inverse();
            newPoints.col(k++) =  points.col(topPoints[i]-1);    //A_KI.transpose()*(oobb.m_minPoint + a.matrix()).eval() ;
            if(topPoints[i] != bottomPoints[i]) {
                //Array3 a(i % gridSize,i/gridSize,0);
                //a.head<2>()*=dxdyInv.inverse();
                newPoints.col(k++) = points.col(bottomPoints[i]-1);  //A_KI.transpose()*(oobb.m_minPoint + a.matrix()).eval() ;
            }
        }
    }
    // Add random points!
    while( k < nPoints) {
        newPoints.col(k++) = points.col( dis(gen) );
    }
}

/**
* Function to optimize oriented bounding box volume.
* Projecting several times into the direction of the axis of the current oobb,
* constructing the mvbb and overwriting the current oobb if volume is smaller
*/
template<typename Derived>
OOBB optimizeMVBB( const MatrixBase<Derived> & points,
                   OOBB oobb, unsigned int times = 10) {

    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived,3,Eigen::Dynamic)


    if( oobb.volume() == 0.0 || times == 0) {
        return oobb;
    }


    bool sameAsCache = true;
    unsigned int cacheIdx = 0; // current write Idx into the cache
    Vector3 dirCache[3]; // save the last three directions (avoiding cycling in choosen axis)

    Vector3 dir;
    ProjectedPointSet proj;
    for(unsigned int loop = 0; loop < times; ++loop ) {

        // Determine Direction (choose x or y axis)
        dir = oobb.getDirection(0);

        // check against all chache values
        for(unsigned char i=0; i<3 && i<loop; ++i) {
            PREC dotp = std::abs(dir.dot(dirCache[i])); //
            if( std::abs(dotp - 1.0) <= 1e-3 ) {
                //std::cout << "Change Dir" << std::endl;
                // direction are almost the same as in the cache, choose another one
                dir = oobb.getDirection(1);
                break;
            }
        }
        // Write to cache and shift write idx
        dirCache[cacheIdx] = dir;
        cacheIdx = (cacheIdx + 1) % 3;

        //std::cout << "Optimizing dir: " << dir << std::endl;

        OOBB o = proj.computeMVBB( dir, points);

        if( o.volume() < oobb.volume()) {
            oobb = o;
            //std::cout << "Volume: " << o.volume() << " dir: " << dir.transpose()<< std::endl;
        }
    }

    return  oobb;
}


/**
* Function to optimize oriented bounding box volume.
* This performs an exhaustive grid search over a given tighly fitted bounding box (use approximateMVBBDiam)
* to find a tighter volume.
* @param gridSize of the 3d Grid
* @param optLoops how many optimization loops are preformed
*        for the oobb computed in the given discrete sampled direction in the grid  (see optimizeMVBB)
*/
template<typename Derived>
OOBB approximateMVBBGridSearch(const MatrixBase<Derived> & points,
                               OOBB oobb,
                               PREC epsilon,
                               const unsigned int gridSize = 5,
                               const unsigned int optLoops = 6
                               ) {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived,3,Eigen::Dynamic)
    //Get the direction of the input OOBB in I frame:
    Vector3 dir1 = oobb.getDirection(0);
    Vector3 dir2 = oobb.getDirection(1);
    Vector3 dir3 = oobb.getDirection(2);

    Vector3 dir;

    ProjectedPointSet proj;

    for(int x = -int(gridSize); x <= (int)gridSize; ++x ) {
        for(int  y = -int(gridSize); y <= (int)gridSize; ++y ) {
            for(int z = 0; z <= (int)gridSize; ++z ) {


                if( MathFunctions::gcd3(x,y,z)> 1 ||  ((x==0) && (y==0) &&  (z==0))  ) {
                    continue;
                }

                // Make direction
                dir = x*dir1 + y*dir2 + z*dir3;
                //std::cout << "dir: " << dir.transpose() << std::endl;

                // Compute MVBB in dirZ
                auto res = proj.computeMVBB(dir,points);

                if(optLoops){
                    res = optimizeMVBB(points,res,optLoops);
                }

                if(res.volume() < oobb.volume()) {
                    oobb = res;
                }

            }
        }
    }

    return oobb;
}

/**
* Function to optimize oriented bounding box volume.
* This constructs an approximation of a tightly fitted bounding box by computing
* the diameter d in 3d and afterwards the projection of the points in the plane perpendicular to direction d
* and then the diameter f in 2d and extruding the OOBB in 2d to the final OOBB approximation in 3d.
*/
template<typename Derived>
OOBB approximateMVBBDiam(const MatrixBase<Derived> & points,
                         const PREC epsilon,
                         const unsigned int optLoops = 10
                        ) {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived,3,Eigen::Dynamic)

    using namespace PointFunctions;

    auto pp = estimateDiameter<3,Vector3>(points,epsilon);

    Vector3 dirZ = pp.first - pp.second;
    if( ( pp.second.array() >=  pp.first.array()).all() ) {
        dirZ *= -1;
    }
    // If direction zero, use (1,0)
    if( (dirZ.array() == 0.0).all() ) {
        dirZ.setZero();
        dirZ(0)= 1;
    }
    std::cout <<"estimated 3d diameter: " << dirZ.transpose() << " eps: " << epsilon << std::endl;


    // Compute MVBB in dirZ
    ProjectedPointSet proj;
    //OOBB oobb = proj.computeMVBB();
    // or faster estimate diameter in projected plane and build coordinate system
    OOBB oobb = proj.computeMVBBApprox(dirZ,points,epsilon);


    if(optLoops) {
        oobb = optimizeMVBB(points,oobb,optLoops);
    }

    return oobb;
}

template<typename Derived>
OOBB approximateMVBB(const MatrixBase<Derived> & points,
                     const PREC epsilon,
                     const unsigned int pointSamples = 400,
                     const unsigned int gridSize = 5,
                     const unsigned int mvbbDiamOptLoops = 2,
                     const unsigned int gridSearchOptLoops = 6
                     ) {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived,3,Eigen::Dynamic)


    // Approx MVBB with Diameter
    auto oobb = approximateMVBBDiam(points,epsilon,mvbbDiamOptLoops);

    if(pointSamples<points.size()) {

        // sample points
        Matrix3Dyn sampled;
        samplePointsGrid(sampled,points,pointSamples,oobb);

        // Exhaustive grid search with sampled points
        oobb = approximateMVBBGridSearch(sampled,oobb,epsilon,gridSize,gridSearchOptLoops);

    } else {
        oobb = approximateMVBBGridSearch(points,oobb,epsilon,gridSize,gridSearchOptLoops);
    }
    return oobb;
}

};



#endif // MVBB_hpp

