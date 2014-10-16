#ifndef  RenderMaterialGenLogic_hpp
#define  RenderMaterialGenLogic_hpp

#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include "LogicNode.hpp"
#include "LookUpTable.hpp"

#include "RenderMaterial.hpp"

#include "RenderConverterData.hpp"

namespace LogicNodes {

    class BodyData : public LogicNode {
    public:

        DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
        DEFINE_MPI_INFORMATION_CONFIG_TYPES;

        struct Inputs {
            enum {
                INPUTS_LAST = 0
            };
        };

        struct Outputs {
            enum {
                BodyId,
                Displacement,
                Velocity,
                MaterialId,
                ProcessId,
                OUTPUTS_LAST
            };
        };

        enum {
            N_INPUTS  = Inputs::INPUTS_LAST,
            N_OUTPUTS = Outputs::OUTPUTS_LAST,
            N_SOCKETS = N_INPUTS + N_OUTPUTS
        };

        DECLARE_OSOCKET_TYPE(BodyId, RigidBodyIdType );
        DECLARE_OSOCKET_TYPE(Displacement, VectorQBody );
        DECLARE_OSOCKET_TYPE(Velocity, VectorUBody );
        DECLARE_OSOCKET_TYPE(MaterialId, unsigned int );
        DECLARE_OSOCKET_TYPE(ProcessId, RankIdType );

        BodyData(unsigned int id) : LogicNode(id) {
            ADD_OSOCK(BodyId,0);
            ADD_OSOCK(Displacement,VectorQBody());
            ADD_OSOCK(Velocity,VectorUBody());
            ADD_OSOCK(MaterialId,0);
            ADD_OSOCK(ProcessId,0);

        }

        virtual ~BodyData() {}

        void compute() {
            // do nothin values are already at the output!
        }
    };

};



#endif
