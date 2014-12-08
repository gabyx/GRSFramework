#ifndef HEADER_28CA05FAD1E29C62
#define HEADER_28CA05FAD1E29C62

/*************************************************************************
 *  -
 *
 * $Id: alloc.h,v 1.1 2002/07/24 12:43:44 greg Exp $
 *
 * Copyright INRIA
 *
 * AUTHOR:
 * Gregoire Malandain (greg@sophia.inria.fr)
 *
 * CREATION DATE:
 * Mon May 15 2000
 *
 *
 * ADDITIONS, CHANGES
 *
 *
 */

#ifndef _alloc_h_
#define _alloc_h_

#include <stdlib.h>
#include <string.h>

namespace ComputeDiameter{

extern void *_AllocateListOfPoints( const int n, const int dim );



typedef struct {
  double *extremity1;
  double *extremity2;
  double squareDiameter;
  int reduction_mode;
} typeSegment;



extern void *_AllocateListOfSegments( const int n );



typedef struct {
  int n;
  int nalloc;
  typeSegment *seg;
} typeListOfSegments;



extern int _AddSegmentToList( typeSegment *s, typeListOfSegments *list );

};
#endif

#endif // header guard
