#ifndef  RenderMaterialGenerator_hpp
#define  RenderMaterialGenerator_hpp

#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include "ExecutionTreeInOut.hpp"

class RenderMaterialGenerator{
    public:

        RenderMaterialGenerator(){}

        ExecutionTreeInOut * getExecTree(){return &m_execTree;}

    private:
        ExecutionTreeInOut m_execTree;
};


#endif // RenderMaterialGenerator_hpp
