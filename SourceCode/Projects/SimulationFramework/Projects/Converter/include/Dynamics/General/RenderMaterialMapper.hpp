#ifndef  RenderMaterialMapper_hpp
#define RenderMaterialMapper_hpp

#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include "ExecutionTreeInOut.hpp"


class BodyDataLogicNode : public LogicNode
{
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES;
    DEFINE_MPI_INFORMATION_CONFIG_TYPES;

    struct Inputs{
        enum{
            INPUTS_LAST = 0
        };
    };

    struct Outputs{
        enum{
            BodyId = Inputs::INPUTS_LAST,
            Displacement,
            Velocity,
            MaterialId,
            ProcessId,
            OUTPUTS_LAST
        };
    };

    enum {
	    N_INPUTS  = Inputs::INPUTS_LAST,
        N_OUTPUTS = Outputs::OUTPUTS_LAST - Inputs::INPUTS_LAST,
        N_SOCKETS = N_INPUTS + N_OUTPUTS
    };

	DECLARE_OSOCKET_TYPE(BodyId, RigidBodyIdType );
	DECLARE_OSOCKET_TYPE(Displacement, VectorQBody );
    DECLARE_OSOCKET_TYPE(Velocity, VectorUBody );
	DECLARE_OSOCKET_TYPE(MaterialId, unsigned int );
    DECLARE_OSOCKET_TYPE(ProcessId, RankIdType );

	BodyDataLogicNode(unsigned int id) : LogicNode(id, N_SOCKETS)
	{
		ADD_OSOCK(BodyId,0);
		ADD_OSOCK(Displacement,VectorQBody());
		ADD_OSOCK(Velocity,VectorUBody());
		ADD_OSOCK(MaterialId,0);
		ADD_OSOCK(ProcessId,0);

	}

	virtual ~BodyDataLogicNode() {}

	void compute(){
        // do nothin values are already at the output!
	}
};

class MaterialMapLogicNode : public LogicNode
{
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES;
    DEFINE_MPI_INFORMATION_CONFIG_TYPES;

    struct Inputs{
        enum{
            Key,
            INPUTS_LAST
        };
    };

    struct Outputs{
        enum{
            Material = Inputs::INPUTS_LAST,
            OUTPUTS_LAST
        };
    };

    enum {
	    N_INPUTS  = Inputs::INPUTS_LAST,
        N_OUTPUTS = Outputs::OUTPUTS_LAST - Inputs::INPUTS_LAST,
        N_SOCKETS = N_INPUTS + N_OUTPUTS
    };

	DECLARE_ISOCKET_TYPE(Key, unsigned int );
	DECLARE_OSOCKET_TYPE(Material, std::string );


	MaterialMapLogicNode(unsigned int id) : LogicNode(id, N_SOCKETS)
	{
		ADD_ISOCK(Key,0);
		ADD_OSOCK(Material,"");
	}

	virtual ~MaterialMapLogicNode() {}

	void compute(){

	}
};


class RenderMaterialMapper{
    public:



        ExecutionTreeInOut<BodyDataLogicNode> & getExecTree(){return m_execTree;}

    private:
        ExecutionTreeInOut<BodyDataLogicNode> m_execTree;
};


#endif // MaterialMapper_hpp
