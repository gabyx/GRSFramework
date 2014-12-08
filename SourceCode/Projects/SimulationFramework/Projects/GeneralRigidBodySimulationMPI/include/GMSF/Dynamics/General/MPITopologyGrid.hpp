#ifndef GMSF_Dynamics_General_MPITopologyGrid_hpp
#define GMSF_Dynamics_General_MPITopologyGrid_hpp

#include "AssertionDebug.hpp"
#include "TypeDefs.hpp"

#include <boost/variant.hpp>
#include <vector>
#include <map>

#include "CartesianGrid.hpp"
#include "AABB.hpp"
#include "Collider.hpp"

#include "MPITopologyVisitors.hpp"

namespace MPILayer{

template< typename ProcessTopologyBase>
class ProcessTopologyGrid : public CartesianGrid<NoCellData> {
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
    DEFINE_MPI_INFORMATION_CONFIG_TYPES

    using RankToAABBType = std::map<unsigned int, AABB >;
    using NeighbourRanksListType = typename ProcessTopologyBase::NeighbourRanksListType;
    using AdjacentNeighbourRanksMapType = typename ProcessTopologyBase::AdjacentNeighbourRanksMapType;

    ProcessTopologyGrid(  NeighbourRanksListType & nbRanks, AdjacentNeighbourRanksMapType & adjNbRanks,
                          RankIdType processRank, unsigned int masterRank,
                          const AABB & aabb,
                          const MyMatrix<unsigned int>::Array3 & dim,
                          bool aligned = true,
                          const Matrix33 & A_IK = Matrix33::Identity()
                          ):
    m_rank(processRank), CartesianGrid<NoCellData>(aabb, dim), m_A_IK(A_IK),
    m_cellNumberingStart(masterRank)
    {
       m_rank = processRank;

        //Initialize neighbours
       nbRanks = getCellNeigbours(m_rank);

        for( auto it = nbRanks.begin(); it!=nbRanks.end(); it++){

            //Initialize adjacent neighbour ranks to m_nbRanks for this neighbour *it
           adjNbRanks[*it] = getCommonNeighbourCells(nbRanks, *it);

            //Get all AABB's of this neighbours
            m_nbAABB[ *it ] = getCellAABB(*it);
        }


        //Get AABB of own rank!
        m_aabb = getCellAABB(m_rank);



    };




    unsigned int getCellRank(const Vector3 & point) const {
        MyMatrix<unsigned int>::Array3 v = CartesianGrid<NoCellData>::getCellIndexClosest(point);
        return getCellRank(v);
    };
    unsigned int getCellRank(const MyMatrix<unsigned int>::Array3 & v) const {
        ASSERTMSG( ( (v(0) >=0 && v(0) < m_dim(0)) && (v(1) >=0 && v(1) < m_dim(1)) && (v(2) >=0 && v(2) < m_dim(2)) ),
                  "Index: " << v << " is out of bound" )

        unsigned int cellRank = v(0) + v(1)*m_dim(0) + v(2) *(m_dim(0)*m_dim(1)) + m_cellNumberingStart;

        ASSERTMSG(cellRank < m_dim(0)*m_dim(1)*m_dim(2) + m_cellNumberingStart
                  && cellRank >= m_cellNumberingStart,
                  "cellRank: " << cellRank <<" not in Dimension: "<< m_dim(0)<<","<< m_dim(1)<<","<< m_dim(2)<<std::endl );
        return cellRank;
    };


    std::set<unsigned int> getCellNeigbours(unsigned int cellRank) const {
        std::set<unsigned int> v;
        // cellRank zero indexed
        ASSERTMSG(cellRank < m_dim(0)*m_dim(1)*m_dim(2) + m_cellNumberingStart
                  && cellRank >= m_cellNumberingStart,
                  "cellRank: " << cellRank <<" not in Dimension: "<< m_dim(0)<<","<< m_dim(1)<<","<< m_dim(2)<<std::endl );

        MyMatrix<unsigned int>::Array3 cell_index = getCellIndex(cellRank);

        MyMatrix<unsigned int>::Array3 ind;

        for(int i=0; i<26; i++) {
            ind(0) = m_nbIndicesOff[i*3+0] + cell_index(0);
            ind(1) = m_nbIndicesOff[i*3+1] + cell_index(1);
            ind(2) = m_nbIndicesOff[i*3+2] + cell_index(2);

            if( ( ind(0) >=0 &&  ind(0) < m_dim(0)) &&
                ( ind(1) >=0 &&  ind(1) < m_dim(1)) &&
                ( ind(2) >=0 &&  ind(2) < m_dim(2)) ) {
                // Add neighbour
                std::pair< typename std::set<unsigned int>::iterator, bool> res =
                        v.insert(getCellRank(ind));
                ASSERTMSG(res.second,"This neighbour number: "<< getCellRank(ind) << " for cell number: "<< cellRank <<" alreads exists!");
            }

        }
        return v;
    };

    /**
    * Gets the common cells between all cellNumbers and the neighbours of cell number cellNumber2
    */
    std::set<unsigned int> getCommonNeighbourCells(const std::set<unsigned int> & cellNumbers,unsigned int cellNumber2) const{
        std::set<unsigned int> nbRanks = getCellNeigbours(cellNumber2);

        std::set<unsigned int> intersec;
        // intersect nbRanks with cellNumbers
        std::set_intersection(cellNumbers.begin(),cellNumbers.end(),nbRanks.begin(),nbRanks.end(),
                      std::inserter(intersec,intersec.begin()));

        return intersec;
    };

    MyMatrix<unsigned int>::Array3 getCellIndex(unsigned int cellRank) const{

        ASSERTMSG(cellRank < m_dim(0)*m_dim(1)*m_dim(2) + m_cellNumberingStart
                  && cellRank >= m_cellNumberingStart,
                  "cellRank: " << cellRank <<" not in Dimension: "<< m_dim(0)<<","<< m_dim(1)<<","<< m_dim(2)<<std::endl );

        MyMatrix<unsigned int>::Array3 v;
        unsigned int cellNumberTemp;

        cellNumberTemp = cellRank;
        v(2) = cellNumberTemp / (m_dim(0)*m_dim(1));

        cellNumberTemp -= v(2)*(m_dim(0)*m_dim(1));
        v(1) = cellNumberTemp / (m_dim(0));

        cellNumberTemp -= v(1)*(m_dim(0));
        v(0) = cellNumberTemp;

        return v;
    };

    /**
    * Gets the AABB, which extends to infinity for boundary cells!
    */
    AABB getCellAABB(unsigned int cellRank) const{

         MyMatrix<unsigned int>::Array3 cell_index = getCellIndex(cellRank);
         Vector3 pL = cell_index.array().cast<PREC>() * m_dxyz.array();
         pL += m_Box.m_minPoint;
         Vector3 pU = (cell_index.array()+1).cast<PREC>()  * m_dxyz.array();
         pU += m_Box.m_minPoint;


        //Expand AABB each axis to max/min if this rank is a boundary cell!
        for(short i = 0;i<3;i++){
            if(cell_index(i) == m_dim(i)-1){
                pU(i) = std::numeric_limits<PREC>::max();
            }
            if( cell_index(i) == 0 ){
                pL(i) = std::numeric_limits<PREC>::lowest();
            }
        }

        return AABB(pL,pU);
    };

    bool checkOverlap(const RigidBodyType * body, NeighbourRanksListType & neighbourProcessRanks, bool & overlapsOwnRank){
        if(m_axisAligned){
            return checkOverlapImpl(m_ColliderAABB,neighbourProcessRanks, overlapsOwnRank, body);
        }
        else{
            return checkOverlapImpl(m_ColliderOOBB,neighbourProcessRanks, overlapsOwnRank, body, m_A_IK );
        }
    }


private:

    template<typename Collider, typename... AddArgs >
    inline bool checkOverlapImpl(Collider & collider,
                                 NeighbourRanksListType & neighbourProcessRanks,
                                 bool & overlapsOwnRank,
                                 const RigidBodyType * body,
                                 AddArgs... args)
    {
            // Check neighbour AABB
            for(auto it = m_nbAABB.begin(); it != m_nbAABB.end(); it++) {
                if( collider.checkOverlap(body,it->second, args...) ) {
                    neighbourProcessRanks.insert(it->first);
                }
            }
            // Check own AABB
            overlapsOwnRank = collider.checkOverlap(body, m_aabb, args...);
            return neighbourProcessRanks.size() > 0;
    }

    unsigned int m_cellNumberingStart;

    template<typename T> friend class TopologyVisitors::BelongsPointToProcess;
    template<typename T> friend class TopologyVisitors::CheckOverlap;

    RankIdType m_rank; ///< Own rank;

    RankToAABBType m_nbAABB; ///< Neighbour AABB in frame G
    AABB m_aabb; ///< Own AABB of this process in frame G

    bool m_axisAligned = true;
    Matrix33 m_A_IK ; ///< The grid can be rotated, this is the transformation matrix from grid frame K to intertia frame I

    ColliderAABB m_ColliderAABB;
    ColliderOOBB m_ColliderOOBB;
};

}; //MPILayer

#endif // MPITopologyGrid_hpp
