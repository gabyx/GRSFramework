#ifndef GRSF_General_GridExtractor_hpp
#define GRSF_General_GridExtractor_hpp

#include <memory>

#include <boost/filesystem.hpp>

#include <H5Cpp.h>

#include "GRSF/Common/TypeDefs.hpp"
#include "GRSF/Common/LogDefines.hpp"
#include "GRSF/Common/SimpleLogger.hpp"
#include "GRSF/Common/EnumClassHelper.hpp"

#include "GRSF/Dynamics/Buffers/RigidBodyState.hpp"

#include "GRSF/Dynamics/General/CartesianGrid.hpp"

#include "GRSF/General/GridExtractionSettings.hpp"

#include "GRSF/Common/HDF5Helpers.hpp"

class GridExtractor{
public:
    DEFINE_LAYOUT_CONFIG_TYPES

    GridExtractor(GridExtractionSettings * settings, Logging::Log * m_log );
    GridExtractor(GridExtractor && g) = default;
    ~GridExtractor();

  /** provide function for SimFileConverter ==================================*/
    void initSimInfo(std::size_t nBodies,std::size_t nStates);
    void initState(boost::filesystem::path folder, std::string filename, double time, unsigned int frameNr);

    template<typename StateContainer>
    void addState(StateContainer & states);

    void finalizeState();

    inline bool isStopBodyLoop(){return false;}
    inline bool isStopFrameLoop(){return false;}
    inline bool isStopFileLoop(){return false;}
    /** ========================================================================*/

    template<typename T>
    using Grid = CartesianGrid< T , std::size_t>;

private:

    void writeGridSettings();

    template<typename TGrid, typename StateContainer>
    void addAllBodies(TGrid * grid, StateContainer & states);

    class CellDataMaxBuffer{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        static const unsigned char maxIdx = 2;

        /* Reset functor */
        template<typename TGrid>
        struct Reset{
            Reset(TGrid * g): m_g(g){}

            template<typename IndexType>
            void operator()(CellDataMaxBuffer & data,
                            IndexType & index) const
            {
                data.m_rigidBodyState = nullptr;
                data.m_distance      = std::numeric_limits<PREC>::lowest();
                data.m_cellCenter = m_g->template getCellPoint<1>(index);
            }


            TGrid * m_g = nullptr;
        };

        template<typename TGrid>
        struct FileSaver{

        };

        /* Gets position in K Frame of Grid*/
        void add(RigidBodyStateAdd * s, Vector3 & K_pos){
            /* add only if greater */
            if(K_pos(maxIdx)>m_distance){
                m_rigidBodyState = s;
                m_distance = K_pos(maxIdx);
            }
        }

        RigidBodyStateAdd * m_rigidBodyState = nullptr;
        PREC m_distance      = std::numeric_limits<PREC>::lowest();
        Vector3 m_cellCenter;
    };

    GridExtractionSettings * m_settings = nullptr;

    /** Grids*/
    using GridType = Grid< CellDataMaxBuffer >;
    std::unique_ptr<GridType> m_grid;


    /** =======================*/

    /** H5 File (row-major storage)*/
    std::unique_ptr<H5::H5File> m_h5File;
    H5::Group m_stateGroup;

    std::size_t m_nBodies;
    std::size_t m_nStates;
    boost::filesystem::path m_folder;
    std::string m_filename;
    double m_time;
    unsigned int m_frameNr;

    /** Log */
    Logging::Log * m_log = nullptr;
};

template<typename StateContainer>
void GridExtractor::addState(StateContainer & states){

    // switch here on grid type

    if(!m_grid){
        m_grid.reset( new GridType(m_settings->m_aabb,m_settings->m_dimension) );
    }

    // reset all cells
    m_grid->applyVisitor( CellDataMaxBuffer::Reset<GridType>(m_grid.get()) );

    addAllBodies(m_grid.get(),states);

    // apply extractor visitor
    m_grid->applyVisitor( m_settings->createDataWriterVisitor(m_grid.get()) );

    // write output
    H5::Group s = m_stateGroup.createGroup("S" + std::to_string(m_frameNr));
    m_settings->writeToHDF5(s);

}

template<typename TGrid, typename StateContainer>
void GridExtractor::addAllBodies(TGrid * grid, StateContainer & states){

    Vector3 K_p;
    for(auto & s : states){

        K_p = grid->getTransformKI() * s.getPosition(); // A_KI * I_pos
        auto * p = grid->template getCellData<false>( K_p );
        if(p){
            p->add(&s,K_p);
        }
    }

}

#endif

