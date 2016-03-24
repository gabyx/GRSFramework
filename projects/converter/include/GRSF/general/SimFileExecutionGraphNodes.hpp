#ifndef GRSF_general_SimFileExecutionGraphNodes_hpp
#define GRSF_general_SimFileExecutionGraphNodes_hpp

#include "GRSF/common/TypeDefs.hpp"
#include "GRSF/common/LogDefines.hpp"

#include LogicTypes_INCLUDE_FILE
#include "GRSF/logic/LogicNode.hpp"

#include "GRSF/dynamics/general/QuaternionHelpers.hpp"
#include "GRSF/dynamics/buffers/RigidBodyState.hpp"
#include "GRSF/dynamics/collision/geometry/AABB.hpp"

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
                SimFilePath,
                NBodies,
                NStates,
                OutputFolder,
                OutputName,
                OutputFilePath, /* the path: "Folder/Name" */
                OUTPUTS_LAST
            };
        };

        enum {
            N_INPUTS  = Inputs::INPUTS_LAST,
            N_OUTPUTS = Outputs::OUTPUTS_LAST,
            N_SOCKETS = N_INPUTS + N_OUTPUTS
        };

        DECLARE_OSOCKET_TYPE(SimFilePath, boost::filesystem::path );
        DECLARE_OSOCKET_TYPE(NBodies, unsigned long int );
        DECLARE_OSOCKET_TYPE(NStates, unsigned long int );

        DECLARE_OSOCKET_TYPE(OutputFolder, boost::filesystem::path );
        DECLARE_OSOCKET_TYPE(OutputName, std::string );
        DECLARE_OSOCKET_TYPE(OutputFilePath, boost::filesystem::path );

        SimFileInfo(unsigned int id) : LogicNode(id) {
            ADD_OSOCK(SimFilePath,"");
            ADD_OSOCK(NBodies,0);
            ADD_OSOCK(NStates,0);
            ADD_OSOCK(OutputFolder,"./"); /* Some folder used for output stuff */
            ADD_OSOCK(OutputName,"SimFileOutput");/* Some name used for output stuff*/
            ADD_OSOCK(OutputFilePath,"./SimFileOutput");
        }

        void setOutput(boost::filesystem::path simFile,
                       boost::filesystem::path filePath,
                       unsigned long int nBodies, unsigned long int nStates
                       )
        {
            if(!filePath.has_filename()){
                filePath /= "SimFileInfoNode-Output";
            }
            SET_OSOCKET_VALUE(SimFilePath,simFile);
            SET_OSOCKET_VALUE(NBodies,nBodies);
            SET_OSOCKET_VALUE(NStates,nStates);

            if(filePath.empty() || !filePath.has_filename()){
                filePath /= "SimFileOutput-f-" + simFile.filename().string();
            }

            SET_OSOCKET_VALUE(OutputFolder,filePath.parent_path());
            SET_OSOCKET_VALUE(OutputName,filePath.filename().string());
            SET_OSOCKET_VALUE(OutputFilePath,filePath);
        }

        ~SimFileInfo(){}
    };


    class StateData: public LogicNode  {
    public:

        struct Inputs {
            enum {
                INPUTS_LAST = 0
            };
        };

        struct Outputs {
            enum {
                Time,
                StateNr,
                OutputFolder,
                OutputName,
                OutputFilePath, /* the path: "Folder/Name" */
                OUTPUTS_LAST
            };
        };

        enum {
            N_INPUTS  = Inputs::INPUTS_LAST,
            N_OUTPUTS = Outputs::OUTPUTS_LAST,
            N_SOCKETS = N_INPUTS + N_OUTPUTS
        };

        DECLARE_OSOCKET_TYPE(Time, double );
        DECLARE_OSOCKET_TYPE(StateNr, unsigned int );
        DECLARE_OSOCKET_TYPE(OutputFolder, boost::filesystem::path );
        DECLARE_OSOCKET_TYPE(OutputName, std::string );
        DECLARE_OSOCKET_TYPE(OutputFilePath, boost::filesystem::path );

        StateData(unsigned int id) : LogicNode(id) {
            ADD_OSOCK(Time,0.0);
            ADD_OSOCK(StateNr,0);
            ADD_OSOCK(OutputFolder,"./");
            ADD_OSOCK(OutputName,"StateOutput-s-0");
            ADD_OSOCK(OutputFilePath,"./StateOutput-s-0");
        }

        void setOutput(boost::filesystem::path filePath,
                       double time, unsigned int stateIdx
                       )
        {

            SET_OSOCKET_VALUE(Time,time);
            SET_OSOCKET_VALUE(StateNr,stateIdx);

            if(filePath.empty() || !filePath.has_filename()){
                filePath /= "StateOutput-s-" + std::to_string(stateIdx);
            }

            SET_OSOCKET_VALUE(OutputFolder,filePath.parent_path());
            SET_OSOCKET_VALUE(OutputName,filePath.filename().string());
            SET_OSOCKET_VALUE(OutputFilePath,filePath);
        }

        ~StateData(){}
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
                ++GET_OSOCKET_REF_VALUE(OverlapCounter);
                DISTRIBUTE_OSOCKET_VALUE(OverlapCounter);
            }
        }

    private:
        Vector3 p;      ///< temporary point
        AABB3d m_aabb;  ///< in K Frame
        Matrix33 m_R_KI; ///< rotation R_KI
    };

};


#endif // GRSF_General_SimFileExecutionGraphLogic_hpp
