#ifndef MPIInformation_hpp
#define MPIInformation_hpp

#include <mpi.h>

#include "AssertionDebug.hpp"
#include "TypeDefs.hpp"

#include "CartesianGrid.hpp"


namespace MPILayer {

template<typename TLayoutConfig>
class ProcessTopology {
public:
    DEFINE_LAYOUT_CONFIG_TYPES_OF(TLayoutConfig);
    virtual bool belongsPointToProcess(const Vector3 & point, unsigned int &neighbourProcessRank) const{
        ERRORMSG("The ProcessTopology::belongsPointToProcess has not been implemented!");
    };
    virtual bool belongsPointToProcess(const Vector3 & point) const{
        ERRORMSG("The ProcessTopology::belongsPointToProcess has not been implemented!");
    };
};

// Prototype
template<typename TLayoutConfig> class ProcessInformation;

template<typename TLayoutConfig>
class ProcessTopologyGrid : public ProcessTopology<TLayoutConfig> {
public:
    DEFINE_LAYOUT_CONFIG_TYPES_OF(TLayoutConfig);

    ProcessTopologyGrid(  const Vector3 & minPoint,
                          const Vector3 & maxPoint,
                          const MyMatrix<unsigned int>::Vector3 & dim,
                          unsigned int processRank): m_grid(minPoint,maxPoint, dim, ProcessInformation<TLayoutConfig>::MASTER_RANK ) {
        m_rank = processRank;
        //Initialize neighbours
        m_nbRanks = m_grid.getCellNeigbours(m_rank);

    };

    bool belongsPointToProcess(const Vector3 & point) const {
        unsigned int nb;
        return belongsPointToProcess(point,nb);
    };


    bool belongsPointToProcess(const Vector3 & point, unsigned int &neighbourProcessRank) const {
        //TODO
        neighbourProcessRank = m_grid.getCellNumber(point);
        if(neighbourProcessRank == m_rank) {
            return true;
        }
        return false;
    };

private:
    unsigned int m_rank; ///< Own rank;
    std::vector<unsigned int> m_nbRanks; ///< Neighbour ranks
    CartesianGrid<TLayoutConfig,NoCellData> m_grid;
};



template<typename TLayoutConfig>
class ProcessInformation {

public:

    DEFINE_LAYOUT_CONFIG_TYPES_OF(TLayoutConfig);

    static const int MASTER_RANK = 0;

    ProcessInformation() {
        m_pProcTopo = NULL;
        initialize();
    }

    ~ProcessInformation() {
        if(m_pProcTopo) {
            delete m_pProcTopo;
        }
    }



     unsigned int getMasterRank() const {
        return MASTER_RANK;
    };

    bool hasMasterRank() const{
        if(m_rank == MASTER_RANK){
            return true;
        }
        return false;
    }

    unsigned int getRank() const {
        return m_rank;
    };
    void setRank(unsigned int rank) {
        m_rank = rank;
    };

    unsigned int getNProcesses() const {
        return m_nProcesses;
    };

    std::string getName() const {
        return m_name;
    };
    void setName(std::string name) {
        m_name = name;
    };

    void createProcTopoGrid(const Vector3 & minPoint,
                            const Vector3 & maxPoint,
                            const MyMatrix<unsigned int>::Vector3 & dim){
        if(m_pProcTopo){
            delete m_pProcTopo;
        }
        m_pProcTopo = new MPILayer::ProcessTopologyGrid<TLayoutConfig>(minPoint,maxPoint,dim, getRank() );
    }

    inline const ProcessTopology<TLayoutConfig> & getProcTopo() const{
        ASSERTMSG(m_pProcTopo,"m_pProcTopo == NULL");
        return *m_pProcTopo;
    };

private:

    void initialize() {
        MPI_Comm_rank(MPI_COMM_WORLD,&this->m_rank);
        MPI_Comm_size(MPI_COMM_WORLD,&this->m_nProcesses);

        std::stringstream s;
        s << "Process_"<<m_rank;
        m_name = s.str();
    };

    ProcessTopology<TLayoutConfig> * m_pProcTopo;

    int m_rank;
    int m_nProcesses;
    std::string m_name;
};




};

#endif

