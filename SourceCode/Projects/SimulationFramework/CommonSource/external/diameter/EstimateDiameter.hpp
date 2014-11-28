#include "diameterUtils/alloc.h"
#include "diameterUtils/rand.h"
#include "diameterUtils/util.h"

namespace ComputeDiameter{

double estimateDiameterInOneList( typeSegment *theDiam,
                                 double **theList,
                                 const int first,
                                 const int last,
                                 const int dim,
                                 double _epsilon_  );

};
