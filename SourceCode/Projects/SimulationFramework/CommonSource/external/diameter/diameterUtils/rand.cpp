/*************************************************************************
 *  -
 *
 * $Id: rand.c,v 1.2 2004/06/10 09:23:33 greg Exp $
 *
 * Copyright INRIA
 *
 * AUTHOR:
 * Gregoire Malandain (greg@sophia.inria.fr)
 *
 * CREATION DATE:
 * Tue May 16 2000
 *
 *
 * ADDITIONS, CHANGES
 *
 *
 */

#include <diameterUtils/rand.h>

namespace ComputeDiameter{

static long int _random_calls_ = 0;
static long int _random_seed_ = 0;

long int _GetRandomCalls()
{
  return( _random_calls_ );
}

long int _GetRandomSeed()
{
  return( _random_seed_ );
}



#ifdef WIN32
void _SetRandomSeed( unsigned int seed )
{
  srand( seed );
  _random_seed_ = seed;
  _random_calls_ = 0;
}
#else
void _SetRandomSeed( long int seed )
{
  srand48( seed );
  _random_seed_ = seed;
  _random_calls_ = 0;
}
#endif



#ifdef WIN32
double _GetRandomDoubleNb( )
{
  _random_calls_ ++;
  return( (double)rand()/(double)RAND_MAX );
}
#else
double _GetRandomDoubleNb( )
{
  _random_calls_ ++;
  return( drand48() );
}
#endif


int _GetRandomIntNb( int min, int max )
{
  if ( min <= max )
    return( (int)(floor( min + _GetRandomDoubleNb()*(double)(max-min+1.0) )) );
  return( (int)(floor( max + _GetRandomDoubleNb()*(double)(min-max+1.0) )) );
}

};
