#ifndef  RenderScriptGeneratorLogic_hpp
#define  RenderScriptGeneratorLogic_hpp

#include <stdio.h>
#include <stdlib.h>

#include <exprtk.hpp>

#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include LogicTypes_INCLUDE_FILE
#include "LogicNode.hpp"

#include "StringFormatNode.hpp"
#include "ConstantNode.hpp"
#include "LookUpTable.hpp"

#include "RigidBodyState.hpp"

#include "RendermanGeometryWriter.hpp"
#include "RenderData.hpp"
#include "QuaternionHelpers.hpp"

namespace LogicNodes {


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

        ~BodyData(){}

        // No initialization
        // No compute function

        void setOutputs(RigidBodyStateAdd * s) {
            SET_OSOCKET_VALUE(BodyId,s->m_id);
            SET_OSOCKET_VALUE(Displacement,s->m_q);
            SET_OSOCKET_VALUE(Velocity,s->m_u);

            if(s->m_data) {
                switch(s->m_data->m_type) {

                case AdditionalBodyData::TypeEnum::PROCESS: {
                    auto * p = static_cast<AdditionalBodyData::Process *>(s->m_data);
                    //std::cout << p->m_processId << std::endl;
                    SET_OSOCKET_VALUE(ProcessId,p->m_processId);
                }
                break;

                case AdditionalBodyData::TypeEnum::PROCESS_MATERIAL: {
                    auto * p = static_cast<AdditionalBodyData::ProcessMaterial *>(s->m_data);
                    SET_OSOCKET_VALUE(MaterialId,p->m_materialId);
                    SET_OSOCKET_VALUE(ProcessId,p->m_processId);
                }
                break;

                default:
                    ERRORMSG("Additional bytes could not be filled into input Node!")
                }
            }
        }

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

    template<typename InType, typename OutType = InType>
    class SimpleFunction : public LogicNode {
    private:
        using ParserT = double;
        using  expression_t = exprtk::expression<ParserT> ;
        exprtk::symbol_table<ParserT> m_symbol_table;
        expression_t m_expression;
        exprtk::parser<ParserT> m_parser;

        unsigned int m_inputs;
    public:

        DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

         struct Outputs {
            enum {
                Out,
                OUTPUTS_LAST
            };
        };

        DECLARE_OSOCKET_TYPE(Out, OutType );

        SimpleFunction(unsigned int id, unsigned int inputs, std::string exprString) : LogicNode(id) {
            m_inputs = inputs;
            for(unsigned int i = 0; i < m_inputs; i++){
                addISock<InType>(InType());
                //std::cout <<"Variables: " << std::string("in")+std::to_string(i) << std::endl;
                m_symbol_table.create_variable(std::string("in")+std::to_string(i));
            }

            ADD_OSOCK(Out,OutType());

            m_expression.register_symbol_table(m_symbol_table);

            m_parser.compile(exprString,m_expression);

        }

        ~SimpleFunction(){}

        // No initialization

        void compute() {
            for(unsigned int i = 0; i < m_inputs; i++){
                m_symbol_table.get_variable(std::string("in")+std::to_string(i))->ref() =
                static_cast<ParserT>(getISocketRefValue<InType>(i));
            }
            //std::cout << "Value: " << m_expression.value() << std::endl;
            SET_OSOCKET_VALUE(Out,static_cast<OutType>(m_expression.value()));
        }

    };


    template<typename IndexType>
    class ColorList : public LogicNode {
    public:

        DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

        struct Inputs {
            enum {
                Enable,
                Index,
                INPUTS_LAST
            };
        };

        struct Outputs {
            enum {
                Color,
                OUTPUTS_LAST
            };
        };

        enum {
            N_INPUTS  = Inputs::INPUTS_LAST,
            N_OUTPUTS = Outputs::OUTPUTS_LAST,
            N_SOCKETS = N_INPUTS + N_OUTPUTS
        };

        DECLARE_ISOCKET_TYPE(Enable, bool );
        DECLARE_ISOCKET_TYPE(Index, IndexType );
        DECLARE_OSOCKET_TYPE(Color, Vector3 );

        ColorList(unsigned int id, unsigned int nColors, unsigned int seed, double amplitude = 1.0) : LogicNode(id) {

            ADD_ISOCK(Enable,true);
            ADD_ISOCK(Index,0);
            ADD_OSOCK(Color,Vector3(0.5,0.5,0.5));

            RandomGenType  gen(seed);
            std::uniform_real_distribution<double> uni(0.0,amplitude);

            if(nColors==0){
                ERRORMSG("nColors needs to be > 0")
            }

            for(unsigned int i=0; i<nColors;i++){
                m_colors.push_back( Vector3( uni(gen), uni(gen), uni(gen) ) );
            }

        }

        ColorList(unsigned int id, std::vector<Vector3> colors) : LogicNode(id) {
            ADD_ISOCK(Enable,true);
            ADD_ISOCK(Index,0);
            ADD_OSOCK(Color,Vector3(0.5,0.5,0.5));

            m_colors = std::move(colors);
        }

        ~ColorList(){}

        // No initialization

        void compute() {
            if(GET_ISOCKET_REF_VALUE(Enable)){
                // Get the indexed color (modulo the size of the list)
                IndexType index = GET_ISOCKET_REF_VALUE(Index) % m_colors.size();
                //std::cout << GET_ISOCKET_REF_VALUE(Index) << std::endl;
                SET_OSOCKET_VALUE(Color,  m_colors[index] );
            }
        }

    private:
        std::vector<Vector3> m_colors;
    };


    class MatteMaterial: public LogicNode {
    public:

        DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

        struct Inputs {
            enum {
                Enable,
                Color,
                INPUTS_LAST
            };
        };

        struct Outputs {
            enum {
                Material,
                OUTPUTS_LAST
            };
        };

        enum {
            N_INPUTS  = Inputs::INPUTS_LAST,
            N_OUTPUTS = Outputs::OUTPUTS_LAST - Inputs::INPUTS_LAST,
            N_SOCKETS = N_INPUTS + N_OUTPUTS
        };
        using RenderMaterialPtr = RenderMaterial*;
        DECLARE_ISOCKET_TYPE(Enable, bool );
        DECLARE_ISOCKET_TYPE(Color, Vector3 );
        DECLARE_OSOCKET_TYPE(Material, RenderMaterialPtr );

        MatteMaterial(unsigned int id) : LogicNode(id) {
            ADD_ISOCK(Enable,true);
            ADD_ISOCK(Color,Vector3(0.5,0.5,0.5));
            ADD_OSOCK(Material,nullptr);
        }

        virtual ~MatteMaterial() {

        }

        virtual void compute(){
            if(GET_ISOCKET_REF_VALUE(Enable)){
                Vector3 & c = GET_ISOCKET_REF_VALUE(Color);
                std::string s = "Color [" + std::to_string(c(0)) + " " + std::to_string(c(1)) + " " + std::to_string(c(2)) + "]\n";
                s.append("Surface \"matte\" \"float Kd\" [1.0] \"float Ka\" [1.0]");
                m_material.setMaterialString( s );
                SET_OSOCKET_VALUE(Material, &m_material);
            }
        }
        virtual void initialize(){}

    private:
        RenderMaterial m_material;
    };


    class RenderScriptWriter : public LogicNode {
    public:

        DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

        struct Inputs {
            enum {
                BodyId,
                BodyPosition,
                BodyOrientation,
                Material,

                FileHeader,
                FileFooter,
                RenderSettings,
                FrameSettings,
                CameraSettings,
                LightSettings,

                Time,
                FrameNr,
                Folder,
                Name,

                INPUTS_LAST
            };
        };

        struct Outputs {
            enum {
                OUTPUTS_LAST = 0
            };
        };

        enum {
            N_INPUTS  = Inputs::INPUTS_LAST,
            N_OUTPUTS = Outputs::OUTPUTS_LAST,
            N_SOCKETS = N_INPUTS + N_OUTPUTS
        };

        using RenderMaterialPtr = RenderMaterial*;
        DECLARE_ISOCKET_TYPE(BodyId, RigidBodyIdType);
        DECLARE_ISOCKET_TYPE(BodyPosition, Vector3 );
        DECLARE_ISOCKET_TYPE(BodyOrientation, Quaternion );
        DECLARE_ISOCKET_TYPE(Material,  RenderMaterialPtr);

        DECLARE_ISOCKET_TYPE(FileHeader, std::string );
        DECLARE_ISOCKET_TYPE(FileFooter,   std::string );
        DECLARE_ISOCKET_TYPE(RenderSettings, std::string );
        DECLARE_ISOCKET_TYPE(FrameSettings, std::string );
        DECLARE_ISOCKET_TYPE(CameraSettings, std::string );
        DECLARE_ISOCKET_TYPE(LightSettings, std::string );

        DECLARE_ISOCKET_TYPE(Time, double );
        DECLARE_ISOCKET_TYPE(FrameNr, unsigned int );
        DECLARE_ISOCKET_TYPE(Folder, boost::filesystem::path );
        DECLARE_ISOCKET_TYPE(Name, std::string);


        RenderScriptWriter(unsigned int id) : LogicNode(id) {
            // Per Body
            ADD_ISOCK(BodyId,0);
            ADD_ISOCK(BodyPosition,Vector3(0,0,0));
            ADD_ISOCK(BodyOrientation,Quaternion(1,0,0,0));
            ADD_ISOCK(Material,nullptr);

            // Per Frame
            ADD_ISOCK(FileHeader,std::string());
            ADD_ISOCK(FileFooter,std::string());
            ADD_ISOCK(RenderSettings,std::string());
            ADD_ISOCK(FrameSettings,std::string());
            ADD_ISOCK(CameraSettings,std::string());
            ADD_ISOCK(LightSettings,std::string());

            // FrameData
            ADD_ISOCK(Time,0.0);
            ADD_ISOCK(FrameNr,0);
            ADD_ISOCK(Folder,"./");
            ADD_ISOCK(Name,"Frame");

        }

        virtual void writeHeader() = 0;
        virtual void writeFrameStart() = 0;

        virtual void writeBody() = 0;

        virtual void writeFrameEnd() = 0;
        virtual void writeFooter() = 0;


        virtual void initFrame() = 0;

        virtual ~RenderScriptWriter(){};
        void compute() {ERRORMSG("Should not be evaluated!")};

    protected:
    };


    class RendermanWriter : public RenderScriptWriter{
    public:
        DEFINE_RENDERCONVERTERDATA_CONFIG_TYPES
        using GeometryMapType = typename RenderDataType::GeometryMapType;

        RendermanWriter(unsigned int id, GeometryMapType * geomMap,
                        bool pipeToSubprocess = false,
                        std::string command="",
                        std::string suffix="" )
            : RenderScriptWriter(id), m_geomMap(geomMap), m_pipeToSubProcess(pipeToSubprocess), m_command(command),
            m_suffix(suffix)
        {
        }

        virtual ~RendermanWriter() {
            m_frameFile.close();
        }

        void writeHeader(){
            std::stringstream s;
            s << GET_ISOCKET_REF_VALUE(FileHeader);
            dumpStream(s);
        }
        void writeFooter(){
            std::stringstream s;
            s << GET_ISOCKET_REF_VALUE(FileFooter);
            dumpStream(s);
        }

        void writeFrameStart(){
            std::stringstream s;
            s << "FrameBegin " << GET_ISOCKET_REF_VALUE(FrameNr) << "\n";
            writeFrameSettings(s);
            writeRenderSettings(s);
            writeCamera(s);
            s << "WorldBegin\n";
            writeLightSettings(s);
            //std::cout << "s: " << s.str()<< std::endl;
            dumpStream(s);
        }
        void writeFrameEnd(){
            std::stringstream s;
            s << "WorldEnd\nFrameEnd\n";
            dumpStream(s);
        }

        void writeBody(){

            // Fill into

            // Write the begin block of the rib file
            static std::stringstream s;

            s.clear();
            s.str("");
            s.seekp(0);
            s.seekg(0);

            s << "AttributeBegin\n";

            // Write position and quaternion
            writePosQuat(s);
            // First get the id of the body then
            RigidBodyIdType & id = GET_ISOCKET_REF_VALUE(BodyId);
            auto geomIt = m_geomMap->find( id );
            if( geomIt == m_geomMap->end() ) {
                ERRORMSG("Geometry for body id: " << id << " not found!")
            }
              // Write the material
            if(GET_ISOCKET_REF_VALUE(Material) == nullptr){
                WARNINGMSG(false,"No material pointer in input at RendermanWriter!")
            }
            else{
                    GET_ISOCKET_REF_VALUE(Material)->write(s);
            }
            // Write the geometry
            m_geomWriter.setStream(&s);
            geomIt->second.apply_visitor(m_geomWriter);

            // finish
            s << "AttributeEnd\n";

            dumpStream(s);
        }

        void initFrame() {

            if( !GET_ISOCKET(Folder)->isConnected() ||
                !GET_ISOCKET(Name)->isConnected() ||
                !GET_ISOCKET(Time)->isConnected() ||
                !GET_ISOCKET(FrameNr)->isConnected()){
                ERRORMSG("RendermanWriter::initFrame --> one of Folder,Name,Time,FrameNr sockets not connected to any FrameData node!")
            }

            // finish last frame
            if(isFileOpen()){
               writeFrameEnd();
               writeFooter();
            }
            // open new frame
            openNewFile();

            // write init
            writeHeader();
            writeFrameStart();
        }

    protected:



        void openNewFile(){

            // open new RIB file at this path and name
            std::string n = GET_ISOCKET_REF_VALUE(Name) + std::to_string(GET_ISOCKET_VALUE(FrameNr)) + ".rib";
            boost::filesystem::path p = GET_ISOCKET_REF_VALUE(Folder) / n;


            if(m_pipeToSubProcess){
                //open pipe to command
                if(m_subprocess){
                    pclose(m_subprocess);
                }
                std::string f = p.string() + m_suffix;
                std::string c = Utilities::stringFormat(m_command,f.c_str());

                m_subprocess = popen( c.c_str() ,"w");
                if(!m_subprocess){
                    ERRORMSG("Subprocess not opened!");
                }

            }else{
                //open file
                m_frameFile.close();
                m_frameFile.open(p.string(),std::ios::trunc | std::ios::out);
                if(!m_frameFile.is_open()) {
                    ERRORMSG("Render frame at : " << p << " could not be opened!")
                }

                m_frameFile << "# Frame for t: " << GET_ISOCKET_REF_VALUE(Time) << "\n";
            }
        }

        bool isFileOpen(){
            if(m_pipeToSubProcess){
                return  m_subprocess;
            }else{
                return m_frameFile.is_open();
            }
        }

        void writeFrameSettings(std::stringstream & s){
            s << GET_ISOCKET_REF_VALUE(FrameSettings);
        }
        void writeCamera(std::stringstream & s){
            s << GET_ISOCKET_REF_VALUE(CameraSettings);
        }
        void writeLightSettings(std::stringstream & s){
            s << GET_ISOCKET_REF_VALUE(LightSettings);
        }
        void writeRenderSettings(std::stringstream & s){
            s << GET_ISOCKET_REF_VALUE(RenderSettings);
        }

        void dumpStream(std::stringstream & s){
            if(m_pipeToSubProcess){
                // Write to subprocess
                fputs(s.str().c_str(),m_subprocess);
            }
            else{
                // Write to file
                m_frameFile << s.rdbuf();
                //ASSERTMSG(m_frameFile.good(),"FUCK"); // stream will fail because s can contain nothing, therefore clear()
                m_frameFile.clear();
            }
        }

        GeometryMapType * m_geomMap;

        std::fstream m_frameFile;

        bool m_pipeToSubProcess;
        std::string m_suffix;
        std::string m_command;
        FILE * m_subprocess;

        RendermanGeometryWriter m_geomWriter;

        void writeBeginHeader(){};
        void writeEndHeader(){};
        void writeCamera(){};

        void writePosQuat(std::stringstream & s) {
            static Matrix44 T = Matrix44::Identity();

            QuaternionHelpers::setRotFromQuaternion(GET_ISOCKET_REF_VALUE(BodyOrientation), T.block<3,3>(0,0) );
            T.col(3).head<3>() = GET_ISOCKET_REF_VALUE(BodyPosition);
              s << "Transform ["
              << T(0,0) << " "
              << T(1,0) << " "
              << T(2,0) << " "
              << T(3,0) << " "

              << T(0,1) << " "
              << T(1,1) << " "
              << T(2,1) << " "
              << T(3,1) << " "

              << T(0,2) << " "
              << T(1,2) << " "
              << T(2,2) << " "
              << T(3,2) << " "

              << T(0,3) << " "
              << T(1,3) << " "
              << T(2,3) << " "
              << T(3,3) << "]\n";
        };


    };

};



#endif
