#ifndef BitCount_hpp
#define BitCount_hpp


namespace BitCount{


    /* ==========================================================================
   Bit Counting routines

   Author: Gurmeet Singh Manku    (manku@cs.stanford.edu)
   Date:   27 Aug 2002
   ========================================================================== */


#include <stdlib.h>
#include <stdio.h>
#include <limits.h>

/* Iterated bitcount iterates over each bit. The while condition sometimes helps
   terminates the loop earlier */
//int iterated_bitcount (unsigned int n)
//{
//    int count=0;
//    while (n)
//    {
//        count += n & 0x1u ;
//        n >>= 1 ;
//    }
//    return count ;
//}


/* Sparse Ones runs proportional to the number of ones in n.
   The line   n &= (n-1)   simply sets the last 1 bit in n to zero. */
//int sparse_ones_bitcount (unsigned int n)
//{
//    int count=0 ;
//    while (n)
//    {
//        count++ ;
//        n &= (n - 1) ;
//    }
//    return count ;
//}


/* Dense Ones runs proportional to the number of zeros in n.
   It first toggles all bits in n, then diminishes count repeatedly */
//int dense_ones_bitcount (unsigned int n)
//{
//    int count = 8 * sizeof(int) ;
//    n ^= (unsigned int) -1 ;
//    while (n)
//    {
//        count-- ;
//        n &= (n - 1) ;
//    }
//    return count ;
//}


/* Precomputed bitcount uses a precomputed array that stores the number of ones
   in each char. */
//static int bits_in_char [256] ;
//
//void compute_bits_in_char (void)
//{
//    unsigned int i ;
//    for (i = 0; i < 256; i++)
//        bits_in_char [i] = iterated_bitcount (i) ;
//    return ;
//}

//int precomputed_bitcount (unsigned int n)
//{
//    // works only for 32-bit ints
//
//    return bits_in_char [n         & 0xffu]
//        +  bits_in_char [(n >>  8) & 0xffu]
//        +  bits_in_char [(n >> 16) & 0xffu]
//        +  bits_in_char [(n >> 24) & 0xffu] ;
//}
//

/* Here is another version of precomputed bitcount that uses a precomputed array
   that stores the number of ones in each short. */

//static char bits_in_16bits [0x1u << 16] ;
//
//void compute_bits_in_16bits (void)
//{
//    unsigned int i ;
//    for (i = 0; i < (0x1u<<16); i++)
//        bits_in_16bits [i] = iterated_bitcount (i) ;
//    return ;
//}

//int precomputed16_bitcount (unsigned int n)
//{
//    // works only for 32-bit int
//
//    return bits_in_16bits [n         & 0xffffu]
//        +  bits_in_16bits [(n >> 16) & 0xffffu] ;
//}




int parallelBitcount (unsigned int n);


/* Nifty  Parallel Count works  the same  way as  Parallel Count  for the
   first three iterations. At the end  of the third line (just before the
   return), each byte of n contains the number of ones in those eight bit
   positions in  the original n. A  little thought then  explains why the
   remainder modulo 255 works. */

//#define MASK_01010101 (((unsigned int)(-1))/3)
//#define MASK_00110011 (((unsigned int)(-1))/5)
//#define MASK_00001111 (((unsigned int)(-1))/17)
//
//int nifty_bitcount (unsigned int n)
//{
//    n = (n & MASK_01010101) + ((n >> 1) & MASK_01010101) ;
//    n = (n & MASK_00110011) + ((n >> 2) & MASK_00110011) ;
//    n = (n & MASK_00001111) + ((n >> 4) & MASK_00001111) ;
//    return n % 255 ;
//}

/* MIT Bitcount

   Consider a 3 bit number as being
        4a+2b+c
   if we shift it right 1 bit, we have
        2a+b
  subtracting this from the original gives
        2a+b+c
  if we shift the original 2 bits right we get
        a
  and so with another subtraction we have
        a+b+c
  which is the number of bits in the original number.

  Suitable masking  allows the sums of  the octal digits  in a 32 bit  number to
  appear in  each octal digit.  This  isn't much help  unless we can get  all of
  them summed together.   This can be done by modulo  arithmetic (sum the digits
  in a number by  molulo the base of the number minus  one) the old "casting out
  nines" trick  they taught  in school before  calculators were  invented.  Now,
  using mod 7 wont help us, because our number will very likely have more than 7
  bits set.   So add  the octal digits  together to  get base64 digits,  and use
  modulo 63.   (Those of you  with 64  bit machines need  to add 3  octal digits
  together to get base512 digits, and use mod 511.)

  This is HACKMEM 169, as used in X11 sources.
  Source: MIT AI Lab memo, late 1970's.
*/
//
//int mit_bitcount(unsigned int n)
//{
//    /* works for 32-bit numbers only */
//    register unsigned int tmp;
//
//    tmp = n - ((n >> 1) & 033333333333) - ((n >> 2) & 011111111111);
//    return ((tmp + (tmp >> 3)) & 030707070707) % 63;
//}

//void verify_bitcounts (unsigned int x)
//{
//    int iterated_ones, sparse_ones, dense_ones ;
//    int precomputed_ones, precomputed16_ones ;
//    int parallel_ones, nifty_ones ;
//    int mit_ones ;
//
//    iterated_ones      = iterated_bitcount      (x) ;
//    sparse_ones        = sparse_ones_bitcount   (x) ;
//    dense_ones         = dense_ones_bitcount    (x) ;
//    precomputed_ones   = precomputed_bitcount   (x) ;
//    precomputed16_ones = precomputed16_bitcount (x) ;
//    parallel_ones      = parallel_bitcount      (x) ;
//    nifty_ones         = nifty_bitcount         (x) ;
//    mit_ones           = mit_bitcount           (x) ;
//
//    if (iterated_ones != sparse_ones)
//    {
//        printf ("ERROR: sparse_bitcount (0x%x) not okay!\n", x) ;
//        exit (0) ;
//    }
//
//    if (iterated_ones != dense_ones)
//    {
//        printf ("ERROR: dense_bitcount (0x%x) not okay!\n", x) ;
//        exit (0) ;
//    }
//
//    if (iterated_ones != precomputed_ones)
//    {
//        printf ("ERROR: precomputed_bitcount (0x%x) not okay!\n", x) ;
//        exit (0) ;
//    }
//
//    if (iterated_ones != precomputed16_ones)
//    {
//        printf ("ERROR: precomputed16_bitcount (0x%x) not okay!\n", x) ;
//        exit (0) ;
//    }
//
//    if (iterated_ones != parallel_ones)
//    {
//        printf ("ERROR: parallel_bitcount (0x%x) not okay!\n", x) ;
//        exit (0) ;
//    }
//
//    if (iterated_ones != nifty_ones)
//    {
//        printf ("ERROR: nifty_bitcount (0x%x) not okay!\n", x) ;
//        exit (0) ;
//    }
//
//    if (mit_ones != nifty_ones)
//    {
//        printf ("ERROR: mit_bitcount (0x%x) not okay!\n", x) ;
//        exit (0) ;
//    }
//
//    return ;
//}


};


#endif
