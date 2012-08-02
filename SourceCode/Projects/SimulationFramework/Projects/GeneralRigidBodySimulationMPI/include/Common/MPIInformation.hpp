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
    virtual bool belongsPointToProcess(const Vector3 & point, unsigned int &neighbourProcessRank){
        ERRORMSG("The ProcessTopology::belongsPointToProcess has not been implemented!");
    };
    virtual bool belongsPointToProcess(const Vector3 & point){
        ERRORMSG("The ProcessTopology::belongsPointToProcess has not been implemented!");
    };
};


template<typename TLayoutConfig>
class ProcessInformation {

public:

    DEFINE_LAYOUT_CONFIG_TYPES_OF(TLayoutConfig);

    static const int MASTER = 0;

    ProcessInformation(){
        initialize();
    }

    void initialize(){
        MPI_Comm_rank(MPI_COMM_WORLD,&this->m_rank);
        MPI_Comm_size(MPI_COMM_WORLD,&this->m_nProcesses);

        std::stringstream s;
        s << "Process_"<<m_rank;
        m_name = s.str();
    };


    unsigned int getRank() {
        return m_rank;
    };
    void setRank(unsigned int rank) {
        m_rank = rank;
    };

    unsigned int getNProcesses() {
        return m_rank;
    };

    std::string getName() {
        return m_rank;
    };
    void setName(std::string name) {
        m_name = name;
    };

    ProcessTopology<TLayoutConfig> m_ProcTopo;

private:
    int m_rank;
    int m_nProcesses;
    std::string m_name;


};


template<typename TLayoutConfig>
class ProcessTopologyGrid : public ProcessTopology<TLayoutConfig> {
public:
    DEFINE_LAYOUT_CONFIG_TYPES_OF(TLayoutConfig);

    ProcessTopologyGrid(  const Vector3 & minPoint,
                          const Vector3 & maxPoint,
                          const MyMatrix<unsigned int>::Vector3 & dim,
                          unsigned int processRank): m_grid(minPoint,maxPoint, dim, ProcessInformation<TLayoutConfig>::MASTER+1 ){
                                m_rank = processRank;
                                //Initialize neighbours
                                m_nbRanks = m_grid.getCellNeigbours(m_rank);

                   };

    bool belongsPointToProcess(const Vector3 & point){
        unsigned int nb;
        return belongsPointToProcess(point,nb);
    };


    bool belongsPointToProcess(const Vector3 & point, unsigned int &neighbourProcessRank){
        //TODO
        neighbourProcessRank = m_grid.getCellNumber(point);
        if(neighbourProcessRank == m_rank){
            return true;
        }
        neighbourProcessRank = -1;
        return false;
    };

private:
    unsigned int m_rank; ///< Own rank;
    std::vector<unsigned int> m_nbRanks; ///< Neighbour ranks
    CartesianGrid<TLayoutConfig,NoCellData> m_grid;
};

};

#endif

