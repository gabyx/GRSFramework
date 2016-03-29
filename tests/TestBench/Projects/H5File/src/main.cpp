// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include <iostream>
#include <string>

#include <H5Cpp.h>
#include "hdf5_hl.h"
#include "hdf5.h"

#ifndef H5_NO_NAMESPACE
    using namespace H5;
#endif

 #include <Eigen/CXX11/Tensor>
using namespace Eigen;

int test1 (void)
{

    const H5std_string  FILE_NAME( "SDS.h5" );
    const H5std_string  DATASET_NAME( "IntArray" );
    const int   NX = 5;                    // dataset dimensions
    const int   NY = 6;
    const int   RANK = 2;
   /*
    * Data initialization.
    */
   int i, j;
   int data[NX][NY];          // buffer for data to write
   for (j = 0; j < NX; j++)
   {
      for (i = 0; i < NY; i++)
     data[j][i] = i + j;
   }
   /*
    * 0 1 2 3 4 5
    * 1 2 3 4 5 6
    * 2 3 4 5 6 7
    * 3 4 5 6 7 8
    * 4 5 6 7 8 9
    */
   // Try block to detect exceptions raised by any of the calls inside it
   try
   {
      /*
       * Turn off the auto-printing when failure occurs so that we can
       * handle the errors appropriately
       */
      Exception::dontPrint();
      /*
       * Create a new file using H5F_ACC_TRUNC access,
       * default file creation properties, and default file
       * access properties.
       */
      H5File file( FILE_NAME, H5F_ACC_TRUNC );
      /*
       * Define the size of the array and create the data space for fixed
       * size dataset.
       */
      hsize_t     dimsf[2];              // dataset dimensions
      dimsf[0] = NX;
      dimsf[1] = NY;
      DataSpace dataspace( RANK, dimsf );
      /*
       * Define datatype for the data in the file.
       * We will store little endian INT numbers.
       */
      IntType datatype( PredType::NATIVE_DOUBLE );
      datatype.setOrder( H5T_ORDER_LE );
      /*
       * Create a new dataset within the file using defined dataspace and
       * datatype and default dataset creation properties.
       */
      DataSet dataset = file.createDataSet( DATASET_NAME, datatype, dataspace );
      /*
       * Write the data to the dataset using default memory space, file
       * space, and transfer properties.
       */
      dataset.write( data, PredType::NATIVE_INT );
   }  // end of try block
   // catch failure caused by the H5File operations
   catch( FileIException error )
   {
      error.printError();
      return -1;
   }
   // catch failure caused by the DataSet operations
   catch( DataSetIException error )
   {
      error.printError();
      return -1;
   }
   // catch failure caused by the DataSpace operations
   catch( DataSpaceIException error )
   {
      error.printError();
      return -1;
   }
   // catch failure caused by the DataSpace operations
   catch( DataTypeIException error )
   {
      error.printError();
      return -1;
   }
   return 0;  // successfully terminated
}

using namespace std;

int test2(){

const H5std_string FILE_NAME( "SDScompound.h5" );
const H5std_string DATASET_NAME( "vel" );
const H5std_string MEMBER1( "a" );
const H5std_string MEMBER2( "b" );
const H5std_string MEMBER3( "c" );
const int   LENGTH = 2;
const int   RANK = 2;
     /* First structure  and dataset*/
   typedef struct s1_t {
    int    a;
    float  b;
    double c;
   } s1_t;
   /* Second structure (subset of s1_t)  and dataset*/
   typedef struct s2_t {
    double c;
    int    a;
   } s2_t;
   // Try block to detect exceptions raised by any of the calls inside it
   try
   {
      /*
       * Initialize the data
       */
      auto s1 = new s1_t[LENGTH][LENGTH];
      for (int i = 0; i< LENGTH; i++){
          for (int j = 0; j< LENGTH; j++)
          {
             s1[i][j].a = i;
             s1[i][j].b = i*i;
             s1[i][j].c = 1./(i+1);
          }
      }
      /*
       * Turn off the auto-printing when failure occurs so that we can
       * handle the errors appropriately
       */
      Exception::dontPrint();
      /*
       * Create the data space.
       */
      hsize_t dim[] = {LENGTH ,LENGTH};   /* Dataspace dimensions */
      DataSpace space( RANK, dim );
      /*
       * Create the file.
       */
      H5File* file = new H5File( FILE_NAME, H5F_ACC_TRUNC );
      /*
       * Create the memory datatype.
       */
      CompType mtype1( sizeof(s1_t) );
      mtype1.insertMember( MEMBER1, HOFFSET(s1_t, a), PredType::NATIVE_INT);
      mtype1.insertMember( MEMBER3, HOFFSET(s1_t, c), PredType::NATIVE_DOUBLE);
      mtype1.insertMember( MEMBER2, HOFFSET(s1_t, b), PredType::NATIVE_FLOAT);
      /*
       * Create the dataset.
       */
      DataSet* dataset;
      dataset = new DataSet(file->createDataSet(DATASET_NAME, mtype1, space));
      /*
       * Write data to the dataset;
       */
      dataset->write( s1, mtype1 );


      {
          hsize_t dim3d[] = {10 ,10, 10,10};   /* Dataspace dimensions */
          DataSpace space3d( 4, dim3d );
          auto d3 = new float[10][10][10][10];
          auto dataset = file->createDataSet("3d", PredType::NATIVE_FLOAT, space3d);
          dataset.write(d3,PredType::NATIVE_FLOAT);
      }


        {

            const int r = 2;
            Tensor<Vector3d,r,RowMajor> t(2,2);
            std::cout << "Tensor size: " << t.size() << std::endl;
            for(int i = 0; i< t.size() ; ++i){
                *static_cast<Vector3d*>(t.data()+i) = Vector3d(i,i,i);
            }
          std::array<hsize_t,r+1> dims;
          for(int i = 0;i<r;++i){
            dims[i] = t.dimension(i);
          }
          dims[r] = 3;
          DataSpace space3d( r+1, &dims[0] );
          auto dataset = file->createDataSet("4dTensor", PredType::NATIVE_DOUBLE, space3d);
          dataset.write(t.data(),PredType::NATIVE_DOUBLE);
          auto id = dataset.getId();
          H5DSset_label(id,0,"GAGADIM");
        }

         {
            const int r = 2;
            Tensor<double,r,RowMajor> t(4,4);
            std::cout << "Tensor size: " << t.size() << std::endl;
            for(int i = 0; i< t.size() ; ++i){
                *static_cast<double*>(t.data()+i) = i;
            }
            std::cout << t << std::endl;
          std::array<hsize_t,r+1> dims;
          for(int i = 0;i<r;++i){
            dims[i] = t.dimension(i);
          }
          dims[r] = 1;
          DataSpace space3d( r+1, &dims[0] );
          auto dataset = file->createDataSet("4dTensor2", PredType::NATIVE_DOUBLE, space3d);
          dataset.write(t.data(),PredType::NATIVE_DOUBLE);
          auto id = dataset.getId();
          H5DSset_label(id,0,"GAGADIM");

          hobj_ref_t rr[10];

          // Make reference data space
          hsize_t dd[1] = {10};
          DataSpace refs( 1, dd);
          auto refset = file->createDataSet("References", PredType::STD_REF_OBJ, refs);
          file->reference(&rr[0],"4dTensor2",H5R_OBJECT);
          refset.write(&rr, PredType::STD_REF_OBJ);
        }
      /*
       * Release resources
       */
      delete dataset;
      delete file;
      /*
       * Open the file and the dataset.
       */
      file = new H5File( FILE_NAME, H5F_ACC_RDONLY );
      dataset = new DataSet (file->openDataSet( DATASET_NAME ));
      /*
       * Create a datatype for s2
       */
      CompType mtype2( sizeof(s2_t) );
      mtype2.insertMember( MEMBER3, HOFFSET(s2_t, c), PredType::NATIVE_DOUBLE);
      mtype2.insertMember( MEMBER1, HOFFSET(s2_t, a), PredType::NATIVE_INT);
      /*
       * Read two fields c and a from s1 dataset. Fields in the file
       * are found by their names "c_name" and "a_name".
       */
      auto s2 = new s2_t[LENGTH][LENGTH];
      dataset->read( s2, mtype2 );
      /*
       * Display the fields
       */
      cout << endl << "Field c : " << endl;
      for (int i = 0; i< LENGTH; i++){
          for (int j = 0; j< LENGTH; j++)
          {
             cout << s2[i][j].c << ",\t";
          }
          cout << endl;
      }
      cout << endl;
      cout << endl << "Field a : " << endl;
      for (int i = 0; i< LENGTH; i++){
          for (int j = 0; j< LENGTH; j++)
          {
             cout << s2[i][j].a << ",\t";
          }
          cout << endl;
      }
      cout << endl;
      /*
       * Create a datatype for s3.
       */
      CompType mtype3( sizeof(float) );
      mtype3.insertMember( MEMBER2, 0, PredType::NATIVE_FLOAT);
      /*
       * Read field b from s1 dataset. Field in the file is found by its name.
       */
      auto s3 = new float[LENGTH][LENGTH]; // Third "structure" - used to read float field of s1
      dataset->read( s3, mtype3 );
      /*
       * Display the field
       */
      cout << endl << "Field b : " << endl;
       for (int i = 0; i< LENGTH; i++){
          for (int j = 0; j< LENGTH; j++)
          {
             cout << s3[i][j] << ",\t";
          }
          cout << endl;
      }
     cout << endl;
      /*
       * Release resources
       */
      delete dataset;
      delete file;


   delete s1;
   delete s2;
   delete s3;
   }  // end of try block
   // catch failure caused by the H5File operations
   catch( FileIException error )
   {
      error.printError();
      return -1;
   }
   // catch failure caused by the DataSet operations
   catch( DataSetIException error )
   {
      error.printError();
      return -1;
   }
   // catch failure caused by the DataSpace operations
   catch( DataSpaceIException error )
   {
      error.printError();
      return -1;
   }
   // catch failure caused by the DataSpace operations
   catch( DataTypeIException error )
   {
      error.printError();
      return -1;
   }

}

void test3(){

#define FILE            "h5ex_t_objref.h5"
#define DATASET         "DS1"
#define DIM0            2

    hid_t       file, space, dset, obj;     /* Handles */
    herr_t      status;
    hsize_t     dims[1] = {DIM0};
    hobj_ref_t  wdata[DIM0],                /* Write buffer */
                *rdata;                     /* Read buffer */
    H5O_type_t  objtype;
    ssize_t     size;
    char        *name;
    int         ndims,
                i;

    /*
     * Create a new file using the default properties.
     */
    file = H5Fcreate (FILE, H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);

    /*
     * Create a dataset with a null dataspace.
     */
    space = H5Screate (H5S_NULL);
    obj = H5Dcreate (file, "DS2", H5T_STD_I32LE, space, H5P_DEFAULT,
                H5P_DEFAULT, H5P_DEFAULT);
    status = H5Dclose (obj);
    status = H5Sclose (space);

    /*
     * Create a group.
     */
    obj = H5Gcreate (file, "G1", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    status = H5Gclose (obj);

    /*
     * Create references to the previously created objects.  Passing -1
     * as space_id causes this parameter to be ignored.  Other values
     * besides valid dataspaces result in an error.
     */
    status = H5Rcreate (&wdata[0], file, "G1", H5R_OBJECT,  -1);
    status = H5Rcreate (&wdata[1], file, "DS2", H5R_OBJECT, -1);

    /*
     * Create dataspace.  Setting maximum size to NULL sets the maximum
     * size to be the current size.
     */
    space = H5Screate_simple (1, dims, NULL);

    /*
     * Create the dataset and write the object references to it.
     */
    dset = H5Dcreate (file, DATASET, H5T_STD_REF_OBJ, space, H5P_DEFAULT,
                H5P_DEFAULT, H5P_DEFAULT);
    status = H5Dwrite (dset, H5T_STD_REF_OBJ, H5S_ALL, H5S_ALL, H5P_DEFAULT,
                wdata);

    /*
     * Close and release resources.
     */
    status = H5Dclose (dset);
    status = H5Sclose (space);
    status = H5Fclose (file);


    /*
     * Now we begin the read section of this example.  Here we assume
     * the dataset has the same name and rank, but can have any size.
     * Therefore we must allocate a new array to read in data using
     * malloc().
     */

    /*
     * Open file and dataset.
     */
    file = H5Fopen (FILE, H5F_ACC_RDONLY, H5P_DEFAULT);
    dset = H5Dopen (file, DATASET, H5P_DEFAULT);

    /*
     * Get dataspace and allocate memory for read buffer.
     */
    space = H5Dget_space (dset);
    ndims = H5Sget_simple_extent_dims (space, dims, NULL);
    rdata = (hobj_ref_t *) malloc (dims[0] * sizeof (hobj_ref_t));

    /*
     * Read the data.
     */
    status = H5Dread (dset, H5T_STD_REF_OBJ, H5S_ALL, H5S_ALL, H5P_DEFAULT,
                rdata);

    /*
     * Output the data to the screen.
     */
    for (i=0; i<dims[0]; i++) {
        printf ("%s[%d]:\n  ->", DATASET, i);

        /*
         * Open the referenced object, get its name and type.
         */
        obj = H5Rdereference (dset, H5R_OBJECT, &rdata[i]);
        status = H5Rget_obj_type (dset, H5R_OBJECT, &rdata[i], &objtype);

        /*
         * Get the length of the name, allocate space, then retrieve
         * the name.
         */
        size = 1 + H5Iget_name (obj, NULL, 0);
        name = (char *) malloc (size);
        size = H5Iget_name (obj, name, size);

        /*
         * Print the object type and close the object.
         */
        switch (objtype) {
            case H5O_TYPE_GROUP:
                printf ("Group");
                break;
            case H5O_TYPE_DATASET:
                printf ("Dataset");
                break;
            case H5O_TYPE_NAMED_DATATYPE:
                printf ("Named Datatype");
        }
        status = H5Oclose (obj);

        /*
         * Print the name and deallocate space for the name.
         */
        printf (": %s\n", name);
        free (name);
    }

    /*
     * Close and release resources.
     */
    free (rdata);
    status = H5Dclose (dset);
    status = H5Sclose (space);
    status = H5Fclose (file);
}
int main(){
    //test1();
    test2();
    //test3();
}
