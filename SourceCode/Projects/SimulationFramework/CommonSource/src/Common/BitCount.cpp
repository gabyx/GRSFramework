#include "BitCount.hpp"



/* Parallel   Count   carries   out    bit   counting   in   a   parallel
   fashion.   Consider   n   after    the   first   line   has   finished
   executing. Imagine splitting n into  pairs of bits. Each pair contains
   the <em>number of ones</em> in those two bit positions in the original
   n.  After the second line has finished executing, each nibble contains
   the  <em>number of  ones</em>  in  those four  bits  positions in  the
   original n. Continuing  this for five iterations, the  64 bits contain
   the  number  of ones  among  these  sixty-four  bit positions  in  the
   original n. That is what we wanted to compute. */

#define TWO(c) (0x1u << (c))
#define MASK(c) (((unsigned int)(-1)) / (TWO(TWO(c)) + 1u))
#define COUNT(x,c) ((x) & MASK(c)) + (((x) >> (TWO(c))) & MASK(c))

int BitCount::parallelBitcount (unsigned int n)
{
    n = COUNT(n, 0) ;
    n = COUNT(n, 1) ;
    n = COUNT(n, 2) ;
    n = COUNT(n, 3) ;
    n = COUNT(n, 4) ;
    /* n = COUNT(n, 5) ;    for 64-bit integers */
    return n ;
}

#undef TWO
#undef MASK
#undef COUNT

