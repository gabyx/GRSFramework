#ifndef  RenderMaterialGenerator_hpp
#define  RenderMaterialGenerator_hpp

#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include "ExecutionTreeInOut.hpp"
#include "RenderMaterial.hpp"

class RenderMaterialGenerator : public ExecutionTreeInOut{
    public:

        RenderMaterialGenerator(){}

        void fillBodyData(){};

        void generateMaterial(){
            execute();
        }
};


#endif // RenderMaterialGenerator_hpp
