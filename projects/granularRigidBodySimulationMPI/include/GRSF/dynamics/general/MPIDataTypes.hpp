// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_general_MPIDataTypes_hpp
#define GRSF_dynamics_general_MPIDataTypes_hpp

#include <mpi.h>

#include "GRSF/common/TypeDefs.hpp"
#include "GRSF/common/AssertionDebug.hpp"

/**
*    Important struct to define all MPI message tags used in this framework!
*/


namespace MPILayer{

    class DataTypes{
        public:

            DEFINE_LAYOUT_CONFIG_TYPES

            template<typename T>
            static MPI_Datatype getDataType(){
                return getDataType_impl<T>();
            }

            template<typename T>
            static void * getDataTypeBuffer(T & t){
                return getDataTypeBuffer_impl<T>(t);
            }

            static void freeAll(){
                MPI_Type_free(&MPIVector3);
            }

            static void commitAll(){
                commitVector3();
            }

        private:

            template<typename T>
            static inline MPI_Datatype getDataType_impl(){
                GRSF_ERRORMSG("This DataType has not been defined!")
                return MPI_DATATYPE_NULL;
            }

            template<typename T>
            static inline void * getDataTypeBuffer_impl(T & t){
                GRSF_ERRORMSG("This DataType has not been defined!")
                return nullptr;
            }


            // Vector3 ===========================================
            static MPI_Datatype MPIVector3;

            static void commitVector3(){
                unsigned int N = 3;
                if(std::is_same<PREC,double>::value){
                    MPI_Type_contiguous(N, MPI_DOUBLE, &MPIVector3);
                }else if(std::is_same<PREC,float>::value){
                    MPI_Type_contiguous(N, MPI_FLOAT, &MPIVector3);
                }else{
                    GRSF_ERRORMSG("This type can not be commited")
                }

                MPI_Type_commit(&MPIVector3);
            }
            // ===================================================
    };

    template<>
    inline MPI_Datatype DataTypes::getDataType_impl<typename DataTypes::Vector3>(){
        return MPIVector3;
    }
    template<>
    inline void * DataTypes::getDataTypeBuffer_impl<typename DataTypes::Vector3>(typename DataTypes::Vector3 & v){
        return v.data();
    }

    class ReduceFunctions{
       public:

       DEFINE_LAYOUT_CONFIG_TYPES

       static MPI_Op MinVector3;
       static MPI_Op MaxVector3;

       static void minMPIVector3(void * invec, void * inoutvec, int * len, MPI_Datatype * t){

            PREC * buffIn  = static_cast<PREC*>(invec);
            PREC * buffOut = static_cast<PREC*>(inoutvec);

            for(int i=0; i<*len;++i){
                typename MyMatrix::MatrixMap<Vector3> o(buffOut);
                o =  typename MyMatrix::MatrixMap<Vector3>(buffIn).cwiseMin(o);

                buffIn+=3;
                buffOut+=3;
            }
        }

       static void maxMPIVector3(void * invec, void * inoutvec, int * len, MPI_Datatype * t){

            PREC * buffIn  = static_cast<PREC*>(invec);
            PREC * buffOut = static_cast<PREC*>(inoutvec);


            for(int i=0; i<*len;++i){
                typename MyMatrix::MatrixMap<Vector3> o(buffOut);
                o =  typename MyMatrix::MatrixMap<Vector3>(buffIn).cwiseMax(o);

                buffIn+=3;
                buffOut+=3;
            }
        }

        static void createAll(){
            MPI_Op_create(&minMPIVector3, true, &MinVector3);
            MPI_Op_create(&maxMPIVector3, true, &MaxVector3);
        }

        static void freeAll(){
            MPI_Op_free(&MinVector3);
            MPI_Op_free(&MaxVector3);
        }

    };

};

#endif
