#ifndef  GRSF_General_SimFileExecutionGraphLogic_hpp
#define  GRSF_General_SimFileExecutionGraphLogic_hpp

#include "GRSF/Common/TypeDefs.hpp"
#include "GRSF/Common/LogDefines.hpp"

#include LogicTypes_INCLUDE_FILE
#include "GRSF/Logic/LogicNode.hpp"

#include "GRSF/Dynamics/General/QuaternionHelpers.hpp"
#include "GRSF/Dynamics/Buffers/RigidBodyState.hpp"
#include "GRSF/Dynamics/Collision/Geometry/AABB.hpp"

namespace LogicNodes {

    class SimFileInfo: public LogicNode  {
    public:

        struct Inputs {
            enum {
                INPUTS_LAST = 0
            };
        };

        struct Outputs {
            enum {
                NBodies,
                NStates,
                OUTPUTS_LAST
            };
        };

        enum {
            N_INPUTS  = Inputs::INPUTS_LAST,
            N_OUTPUTS = Outputs::OUTPUTS_LAST,
            N_SOCKETS = N_INPUTS + N_OUTPUTS
        };

        DECLARE_OSOCKET_TYPE(NBodies, unsigned long int );
        DECLARE_OSOCKET_TYPE(NStates, unsigned long int );

        SimFileInfo(unsigned int id) : LogicNode(id) {
            ADD_OSOCK(NBodies,0);
            ADD_OSOCK(NStates,0);
        }

        void setOutput(unsigned long int nBodies, unsigned long int nStates){
            SET_OSOCKET_VALUE(NBodies,nBodies);
            SET_OSOCKET_VALUE(NStates,nStates);
        }

        ~SimFileInfo(){}
    };


    class FrameData: public LogicNode  {
    public:

        struct Inputs {
            enum {
                INPUTS_LAST = 0
            };
        };

        struct Outputs {
            enum {
                Time,
                FrameNr,
                Folder,
                Name,
                OUTPUTS_LAST
            };
        };

        enum {
            N_INPUTS  = Inputs::INPUTS_LAST,
            N_OUTPUTS = Outputs::OUTPUTS_LAST,
            N_SOCKETS = N_INPUTS + N_OUTPUTS
        };

        DECLARE_OSOCKET_TYPE(Time, double );
        DECLARE_OSOCKET_TYPE(FrameNr, unsigned int );
        DECLARE_OSOCKET_TYPE(Folder, boost::filesystem::path );
        DECLARE_OSOCKET_TYPE(Name, std::string );

        FrameData(unsigned int id) : LogicNode(id) {
            ADD_OSOCK(Time,0.0);
            ADD_OSOCK(FrameNr,0);
            ADD_OSOCK(Folder,"./");
            ADD_OSOCK(Name,"Frame");
        }

        void setOutput(boost::filesystem::path folder, std::string filename, double time, unsigned int frameNr){
            SET_OSOCKET_VALUE(Folder,folder);
            SET_OSOCKET_VALUE(Name,filename);
            SET_OSOCKET_VALUE(Time,time);
            SET_OSOCKET_VALUE(FrameNr,frameNr);
        }

        ~FrameData(){}
    };



    namespace ABD = AdditionalBodyData;

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
                OverlapTotal,
                GeomId,
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
        DECLARE_OSOCKET_TYPE(OverlapTotal, PREC );
        DECLARE_OSOCKET_TYPE(GeomId, unsigned int );

        BodyData(unsigned int id) : LogicNode(id) {
            ADD_OSOCK(BodyId,0);
            ADD_OSOCK(Displacement,VectorQBody());
            ADD_OSOCK(Velocity,VectorUBody());
            ADD_OSOCK(MaterialId,0);
            ADD_OSOCK(ProcessId,0);
            ADD_OSOCK(OverlapTotal,0.0);
            ADD_OSOCK(GeomId,0);
        }

        ~BodyData(){}

        // No initialization
        // No compute function

        void setOutputs(RigidBodyStateAdd * s) {

            static AddBytesVisitor vis(this);
            SET_OSOCKET_VALUE(BodyId,s->m_id);
            SET_OSOCKET_VALUE(Displacement,s->m_q);
            SET_OSOCKET_VALUE(Velocity,s->m_u);

            if(s->m_data) {
                s->m_data->applyVisitor(vis);
            }
        }
    private:

        struct AddBytesVisitor{
            AddBytesVisitor(BodyData * p): m_p(p){};

            void operator()(ABD::AddBytes<EnumConversion::toIntegral(ABD::TypeEnum::PROCESS)> * add){
                 SET_OSOCKET_VALUE_PTR(m_p,ProcessId,add->m_processId);
            }
            void operator()(ABD::AddBytes<EnumConversion::toIntegral(ABD::TypeEnum::PROCESS_MATERIAL)> * add){
                 SET_OSOCKET_VALUE_PTR(m_p,ProcessId, add->m_processId);
                 SET_OSOCKET_VALUE_PTR(m_p,MaterialId,add->m_materialId);
            }
            void operator()(ABD::AddBytes<EnumConversion::toIntegral(ABD::TypeEnum::PROCESS_MATERIAL_OVERLAP)> * add){
                 SET_OSOCKET_VALUE_PTR(m_p,ProcessId, add->m_processId);
                 SET_OSOCKET_VALUE_PTR(m_p,MaterialId,add->m_materialId);
                 SET_OSOCKET_VALUE_PTR(m_p,OverlapTotal,add->m_overlapTotal);
            }
            void operator()(ABD::AddBytes<EnumConversion::toIntegral(ABD::TypeEnum::PROCESS_MATERIAL_OVERLAP_GLOBGEOMID)> * add){
                 SET_OSOCKET_VALUE_PTR(m_p,ProcessId, add->m_processId);
                 SET_OSOCKET_VALUE_PTR(m_p,MaterialId,add->m_materialId);
                 SET_OSOCKET_VALUE_PTR(m_p,OverlapTotal,add->m_overlapTotal);
                 SET_OSOCKET_VALUE_PTR(m_p,GeomId,add->m_geomId);
            }
            void operator()(ABD::AddBytes<EnumConversion::toIntegral(ABD::TypeEnum::PROCESS_OVERLAP)> * add){
                 SET_OSOCKET_VALUE_PTR(m_p,ProcessId,   add->m_processId);
                 SET_OSOCKET_VALUE_PTR(m_p,OverlapTotal,add->m_overlapTotal);
            }
            template<typename T>
            void operator()(T * p){
                 ERRORMSG("Additional bytes could not be filled into input Node!")
            }

            BodyData * m_p;
        };

    };

    class DisplacementToPosQuat : public LogicNode {
    public:

        DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
        DEFINE_MPI_INFORMATION_CONFIG_TYPES;

        struct Inputs {
            enum {
                Displacement,
                INPUTS_LAST
            };
        };

        struct Outputs {
            enum {
                Position,
                Quaternion,
                OUTPUTS_LAST
            };
        };

        enum {
            N_INPUTS  = Inputs::INPUTS_LAST,
            N_OUTPUTS = Outputs::OUTPUTS_LAST,
            N_SOCKETS = N_INPUTS + N_OUTPUTS
        };

        DECLARE_ISOCKET_TYPE(Displacement, VectorQBody );
        DECLARE_OSOCKET_TYPE(Position, Vector3 );
        DECLARE_OSOCKET_TYPE(Quaternion, Quaternion );

        DisplacementToPosQuat(unsigned int id) : LogicNode(id) {
            ADD_ISOCK(Displacement,VectorQBody());
            ADD_OSOCK(Position,Vector3(0,0,0));
            ADD_OSOCK(Quaternion,Quaternion(1,0,0,0));
        }

        ~DisplacementToPosQuat() {}

        // No initialization

        void compute() {
            // convert displacement q into position and quaternion
            SET_OSOCKET_VALUE(Position,  GET_ISOCKET_REF_VALUE(Displacement).head<3>() );
            SET_OSOCKET_VALUE(Quaternion,  GET_ISOCKET_REF_VALUE(Displacement).tail<4>() );
        }
    };

    class VelocityToVelRot : public LogicNode {
    public:

        DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
        DEFINE_MPI_INFORMATION_CONFIG_TYPES;

        struct Inputs {
            enum {
                Vel,
                INPUTS_LAST
            };
        };

        struct Outputs {
            enum {
                TransVel,
                RotVel,
                OUTPUTS_LAST
            };
        };

        enum {
            N_INPUTS  = Inputs::INPUTS_LAST,
            N_OUTPUTS = Outputs::OUTPUTS_LAST,
            N_SOCKETS = N_INPUTS + N_OUTPUTS
        };

        DECLARE_ISOCKET_TYPE(Vel, VectorUBody );
        DECLARE_OSOCKET_TYPE(TransVel, Vector3 );
        DECLARE_OSOCKET_TYPE(RotVel, Vector3 );

        VelocityToVelRot(unsigned int id) : LogicNode(id) {
            ADD_ISOCK(Vel,VectorUBody());
            ADD_OSOCK(TransVel,Vector3(0,0,0));
            ADD_OSOCK(RotVel, Vector3(0,0,0));
        }

        ~VelocityToVelRot() {}

        // No initialization

        void compute() {
            // convert displacement q into position and quaternion
            SET_OSOCKET_VALUE(TransVel,  GET_ISOCKET_REF_VALUE(Vel).head<3>() );
            SET_OSOCKET_VALUE(RotVel,  GET_ISOCKET_REF_VALUE(Vel).tail<3>() );
        }
    };

    class OOBBCollider : public LogicNode {
    public:

        DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
        DEFINE_MPI_INFORMATION_CONFIG_TYPES;

        struct Inputs {
            enum {
                Pos, ///< Position in I Frame
                INPUTS_LAST
            };
        };

        struct Outputs {
            enum {
                OverlapCounter,
                OUTPUTS_LAST
            };
        };

        enum {
            N_INPUTS  = Inputs::INPUTS_LAST,
            N_OUTPUTS = Outputs::OUTPUTS_LAST,
            N_SOCKETS = N_INPUTS + N_OUTPUTS
        };

        DECLARE_ISOCKET_TYPE(Pos, Vector3 );
        DECLARE_OSOCKET_TYPE(OverlapCounter, unsigned int );

        OOBBCollider(unsigned int id,
                     AABB3d & aabb,
                     Matrix33 & R_KI) : LogicNode(id), m_aabb(aabb), m_R_KI(R_KI)
        {
            ADD_ISOCK(Pos, Vector3(0,0,0));
            ADD_OSOCK(OverlapCounter, 0);
        }

        ~OOBBCollider() {}

        void reset(){
            SET_OSOCKET_VALUE(OverlapCounter,0);
        }

        void compute() {
            p = m_R_KI.transpose() * GET_ISOCKET_REF_VALUE(Pos); // A_KI * I_pos
            if( m_aabb.overlaps( p ) ){
                ++ GET_OSOCKET_REF_VALUE(OverlapCounter);
            }
        }

    private:
        Vector3 p;      ///< temporary point
        AABB3d m_aabb;  ///< in K Frame
        Matrix33 m_R_KI; ///< rotation R_KI
    };

};


#endif // GRSF_General_SimFileExecutionGraphLogic_hpp
