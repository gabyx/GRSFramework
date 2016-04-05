// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include  <stdlib.h>
#include  <stdio.h>
#include  <assert.h>
#include  <memory.h>
#include  <math.h>






#include  "gdiam.hpp"

/*--- Start of Code ---*/

namespace GDIAM{


void   test_itself( gdiam_real  * points, int  num )
{
    GPointPair   pair;

    printf( "Computing the diameter for %d points selected "
            "uniformly from the unit cube\n", num );
    pair = gdiam_approx_diam_pair( (gdiam_real *)points, num, 0.0 );
    printf( "Diameter distance: %g\n", pair.distance );
    printf( "Points realizing the diameter\n"
            "\t(%g, %g, %g) - (%g, %g, %g)\n",
            pair.p[ 0 ], pair.p[ 1 ], pair.p[ 2 ],
            pair.q[ 0 ], pair.q[ 1 ], pair.q[ 2 ] );


    gdiam_point  * pnt_arr;
    gdiam_bbox   bb;

    pnt_arr = gdiam_convert( (gdiam_real *)points, num );

    printf( "Computing a tight-fitting bounding box of the point-set\n" );
    bb = gdiam_approx_mvbb_grid_sample( pnt_arr, num, 5, 400 );

    printf( "Resulting bounding box:\n" );
    bb.dump();

    printf( "Axis parallel bounding box\n" );
    GBBox   bbx;
    bbx.init();
    for  ( int  ind = 0; ind < num; ind++ )
        bbx.bound( points + (ind * 3) );
    bbx.dump();

    free(pnt_arr);

}


void   standard_test()
{
    gdiam_real  * points;
    int  num;

    num = 1000000*5;

    points = (gdiam_point)malloc( sizeof( gdiam_point_t ) * num );

    assert( points != NULL );

    // Pick randomly points from the unit cube */
    for  ( int  ind = 0; ind < num; ind++ ) {
        points[ ind * 3 + 0 ] = drand48();
        points[ ind * 3 + 1 ] = drand48();
        points[ ind * 3 + 2 ] = drand48();
    }

    test_itself( points, num );
    free(points);
}


void  read_points( FILE   * fl, gdiam_real  * points, int  points_num )
{
    int  args;
    double  x, y, z;

    for  ( int  ind = 0; ind < points_num; ind++ ) {
        args = fscanf( fl, "%lg %lg %lg\n", &x, &y, &z );
        assert( args == 3 );
        points[ ind * 3 + 0 ] = x;
        points[ ind * 3 + 1 ] = y;
        points[ ind * 3 + 2 ] = z;
    }
}


void  test_file( const char  * file_name )
{
    gdiam_real  * points;
    FILE   * fl;
    int  args, points_num;

    fl = fopen( file_name, "rt" );
    if  ( fl == NULL ) {
        printf( "Unable to open file: [%s]\n", file_name );
        exit( -1 );
    }
    args = fscanf( fl, "%d\n", &points_num );
    assert( ( args > 0 )  &&  ( points_num > 0 ) );

    points = (gdiam_point)malloc( sizeof( gdiam_point_t ) * points_num );
    assert( points != NULL );

    read_points( fl, points, points_num );
    fclose( fl );

    test_itself( points, points_num );
}


int  test( int  argc, char  ** argv )
{
    if  ( argc == 1 ) {
        standard_test();
        return  0;
    }

    for  ( int  ind = 1; ind < argc; ind++ )
        test_file( argv[ ind ] );

    return  0;
}

};
