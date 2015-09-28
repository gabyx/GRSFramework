#ifndef  RenderExecutionGraphLogic_hpp
#define  RenderExecutionGraphLogic_hpp

#include "GRSF/Common/TypeDefs.hpp"
#include "GRSF/Common/LogDefines.hpp"

#include LogicTypes_INCLUDE_FILE
#include "GRSF/Logic/LogicNode.hpp"

#include "GRSF/General/RendermanGeometryWriter.hpp"
#include "GRSF/General/RenderData.hpp"


#include "GRSF/Common/ColorGradient.hpp"

namespace LogicNodes {

    template<typename IndexType>
    class ColorList : public LogicNode {
    public:

        DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

        using Vector3Vec = StdVecAligned<Vector3>;

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


    class ColorGradientNode: public LogicNode, public ColorGradient {
    public:

        DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

        struct Inputs {
            enum {
                Enable,
                Value,
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
        DECLARE_ISOCKET_TYPE(Value, double );
        DECLARE_OSOCKET_TYPE(Color, Vector3 );

        ColorGradientNode(unsigned int id, PREC min, PREC max) : LogicNode(id), m_min(min), m_max(max) {

            ADD_ISOCK(Enable,true);
            ADD_ISOCK(Value,0);
            ADD_OSOCK(Color,Vector3(0.5,0.5,0.5));
        }
        ~ColorGradientNode(){}

        // No initialization

        void compute() {
            if(GET_ISOCKET_REF_VALUE(Enable)){
                PREC index = (GET_ISOCKET_REF_VALUE(Value) - m_min)/(m_max-m_min);
                index = std::max(std::min(1.0,index),0.0);
                //std::cout <<GET_ISOCKET_REF_VALUE(Value) << std::endl;
                this->getColorAtValue(index,GET_OSOCKET_REF_VALUE(Color));
            }
        }
    private:

        PREC m_min, m_max;

    };


    class MatteMaterial: public LogicNode {
    public:

        DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

        struct Inputs {
            enum {
                Enable,
                Color,
                Ka,
                Kd,
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
        DECLARE_ISOCKET_TYPE(Ka, double);
        DECLARE_ISOCKET_TYPE(Kd, double);
        DECLARE_OSOCKET_TYPE(Material, RenderMaterialPtr );

        MatteMaterial(unsigned int id, const Vector3 & color = Vector3(0.5,0.5,0.5),
                      double ka = 1.0, double kd = 1.0 ) : LogicNode(id) {
            ADD_ISOCK(Enable,true);
            ADD_ISOCK(Color,color);
            ADD_ISOCK(Ka,ka);
            ADD_ISOCK(Kd,kd);
            ADD_OSOCK(Material,nullptr);
        }

        virtual ~MatteMaterial() {

        }
        virtual void compute(){
            if(GET_ISOCKET_REF_VALUE(Enable)){
                m_material.clear();
                m_material << "Color [" << GET_ISOCKET_REF_VALUE(Color).format(MyMatrixIOFormat::SpaceSep) << "]\n"
                     << "Surface \"matte\" \"float Kd\" [" << std::to_string(GET_ISOCKET_REF_VALUE(Kd)) << "] \"float Ka\" ["
                     << std::to_string(GET_ISOCKET_REF_VALUE(Ka)) << " ]\nAttribute \"visibility\" \"transmission\" 1";

                SET_OSOCKET_VALUE(Material, &m_material);
            }
        }
        virtual void initialize(){}

    private:
        RenderMaterial m_material;
    };

    class BxdfDisneyMaterial: public LogicNode {
    public:

        DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

        struct Inputs {
            enum {
                Enable,
                BaseColor,
                EmitColor,
                SubsurfaceColor,
                Subsurface,
                Metallic,
                Specular,
                SpecularTint,
                Roughness,
                Anisotropic,
                Sheen,
                SheenTint,
                Clearcoat,
                ClearcoatGloss,
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
        DECLARE_ISOCKET_TYPE(BaseColor, Vector3 );
        DECLARE_ISOCKET_TYPE(EmitColor, Vector3);
        DECLARE_ISOCKET_TYPE(SubsurfaceColor, Vector3);
        DECLARE_ISOCKET_TYPE(Subsurface, double);
        DECLARE_ISOCKET_TYPE(Metallic, double);
        DECLARE_ISOCKET_TYPE(Specular, double);
        DECLARE_ISOCKET_TYPE(SpecularTint, double);
        DECLARE_ISOCKET_TYPE(Roughness, double);
        DECLARE_ISOCKET_TYPE(Anisotropic, double);
        DECLARE_ISOCKET_TYPE(Sheen, double);
        DECLARE_ISOCKET_TYPE(SheenTint, double);
        DECLARE_ISOCKET_TYPE(Clearcoat, double );
        DECLARE_ISOCKET_TYPE(ClearcoatGloss, double );

        DECLARE_OSOCKET_TYPE(Material, RenderMaterialPtr );

        BxdfDisneyMaterial(unsigned int id,
                      const Vector3 & baseColor = Vector3(0.5,0.5,0.5),
                      const Vector3 & emitColor = Vector3(0.0,0.0,0.0),
                      const Vector3 & subsurfaceColor = Vector3(0.0,0.0,0.0),
                      double subsurface = 0.0,
                      double metallic = 0.0,
                      double specular = 0.5,
                      double specularTint = 0.0,
                      double roughness = 0.25,
                      double anisotropic = 0.0,
                      double sheen = 0.0,
                      double sheenTint = 0.5,
                      double clearcoat = 0.0,
                      double clearcoatGloss = 1.0) : LogicNode(id) {
            ADD_ISOCK(Enable,true);
            ADD_ISOCK(BaseColor,baseColor);
            ADD_ISOCK(EmitColor,emitColor);
            ADD_ISOCK(SubsurfaceColor,subsurfaceColor);
            ADD_ISOCK(Subsurface,subsurface);
            ADD_ISOCK(Metallic,metallic);
            ADD_ISOCK(Specular,specular);
            ADD_ISOCK(SpecularTint,specularTint);
            ADD_ISOCK(Roughness,roughness);
            ADD_ISOCK(Anisotropic,anisotropic);
            ADD_ISOCK(Sheen,sheen);
            ADD_ISOCK(SheenTint,sheenTint);
            ADD_ISOCK(Clearcoat,clearcoat);
            ADD_ISOCK(ClearcoatGloss,clearcoatGloss);
            ADD_OSOCK(Material,nullptr);
        }

        virtual ~BxdfDisneyMaterial(){}

        virtual void compute(){
            if(GET_ISOCKET_REF_VALUE(Enable)){
                m_material.clear();
                m_material << "Bxdf \"PxrDisney\" \"0\" "
                          <<" \"color baseColor\" [" << GET_ISOCKET_REF_VALUE(BaseColor).transpose().format(MyMatrixIOFormat::SpaceSep) << "]"
                          <<" \"color emitColor\" [" << GET_ISOCKET_REF_VALUE(EmitColor).transpose().format(MyMatrixIOFormat::SpaceSep) << "]"
                          <<" \"color subsurfaceColor\" [" << GET_ISOCKET_REF_VALUE(SubsurfaceColor).transpose().format(MyMatrixIOFormat::SpaceSep) << "]"
                          <<" \"float subsurface\" [" << GET_ISOCKET_REF_VALUE(Subsurface) << "]"
                          <<" \"float metallic\" ["  << GET_ISOCKET_REF_VALUE(Metallic)<< "]"
                          <<" \"float specular\" ["  << GET_ISOCKET_REF_VALUE(Specular)<< "]"
                          <<" \"float specularTint\" [" << GET_ISOCKET_REF_VALUE(SpecularTint) << "]"
                          <<" \"float roughness\" ["  << GET_ISOCKET_REF_VALUE(Roughness)<< "]"
                          <<" \"float anisotropic\" ["  << GET_ISOCKET_REF_VALUE(Anisotropic)<< "]"
                          <<" \"float sheen\" ["  << GET_ISOCKET_REF_VALUE(Sheen)<< "]"
                          <<" \"float sheenTint\" ["  << GET_ISOCKET_REF_VALUE(SheenTint)<< "]"
                          <<" \"float clearcoat\" ["  << GET_ISOCKET_REF_VALUE(Clearcoat)<< "]"
                          <<" \"float clearcoatGloss\" [" << GET_ISOCKET_REF_VALUE(ClearcoatGloss)<< "]";
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
                BodiesBegin,
                BodiesEnd,

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
        DECLARE_ISOCKET_TYPE(BodiesBegin, std::string );
        DECLARE_ISOCKET_TYPE(BodiesEnd, std::string );

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
            ADD_ISOCK(BodiesBegin,std::string());
            ADD_ISOCK(BodiesEnd,std::string());

            // FrameData
            ADD_ISOCK(Time,0.0);
            ADD_ISOCK(FrameNr,0);
            ADD_ISOCK(Folder,"./");
            ADD_ISOCK(Name,"Frame");

        }

        virtual void writeHeader() = 0;

        virtual void writeBodiesBegin() = 0;
        virtual void writeBody() = 0;
        virtual void writeBodiesEnd() = 0;

        virtual void writeFooter() = 0;


        virtual void initFrame() = 0;
        virtual void finalizeFrame() = 0;

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
                        std::string command="")
            : RenderScriptWriter(id), m_geomMap(geomMap), m_pipeToSubProcess(pipeToSubprocess), m_command(command)
        {
        }

        virtual ~RendermanWriter() {

            if(m_pipeToSubProcess){
                if(m_subprocess){
                pclose(m_subprocess);
                }
            }else{
                m_frameFile.close();
            }
        }

        void writeHeader(){
            m_s << GET_ISOCKET_REF_VALUE(FileHeader);
        }
        void writeFooter(){
            m_s << GET_ISOCKET_REF_VALUE(FileFooter);
        }

        void writeBodiesBegin(){
            m_s << GET_ISOCKET_REF_VALUE(BodiesBegin);
        }
        void writeBody(){

            m_s << "AttributeBegin\n";

            // Write position and quaternion
            writePosQuat(m_s);
            // First get the id of the body then
            RigidBodyIdType & id = GET_ISOCKET_REF_VALUE(BodyId);
            auto geomIt = m_geomMap->find( id );
            if( geomIt == m_geomMap->end() ) {
                ERRORMSG("Geometry for body id: " << RigidBodyId::getBodyIdString(id) << " not found!")
            }
              // Write the material
            if(GET_ISOCKET_REF_VALUE(Material) == nullptr){
                WARNINGMSG(false,"No material pointer in input at RendermanWriter!")
            }
            else{
                    GET_ISOCKET_REF_VALUE(Material)->write(m_s);
            }
            // Write the geometry
            m_geomWriter.setStream(&m_s);
            geomIt->second.apply_visitor(m_geomWriter);

            // finish
            m_s << "AttributeEnd\n";

            dumpStream();
        }

        void writeBodiesEnd(){
            m_s << GET_ISOCKET_REF_VALUE(BodiesEnd);
        }

        void initFrame() {

            m_s.str("");

            if( !GET_ISOCKET(Folder)->isConnected() ||
                !GET_ISOCKET(Name)->isConnected() ||
                !GET_ISOCKET(Time)->isConnected() ||
                !GET_ISOCKET(FrameNr)->isConnected()){
                ERRORMSG("RendermanWriter::initFrame --> one of Folder,Name,Time,FrameNr sockets not connected to any FrameData node!")
            }

            // open new frame
            openNewFile();

            // write init
            writeHeader();
            writeBodiesBegin();
            dumpStream();
        }

        void finalizeFrame(){
            // finish last frame
            if(isFileOpen()){
               writeBodiesEnd();
               writeFooter();
               dumpStream();
            }

            closeFile();
        }

    protected:



        void openNewFile(){

            // open new RIB file at this path and name

            boost::filesystem::path f = GET_ISOCKET_REF_VALUE(Name);
            if(f.extension() != ".rib" ){
                f += ".rib";
            }

            boost::filesystem::path p = GET_ISOCKET_REF_VALUE(Folder) / f ;


            if(m_pipeToSubProcess){
                //open pipe to command
                if(m_subprocess){
                    pclose(m_subprocess);
                }
                std::string f = p.string();
                std::string c;
                try{
                    c = Utilities::stringFormat(m_command,f.c_str());
                }
                catch(...){
                    // if we could not format the command with the file, dont do anything
                    c = m_command;
                }
                m_subprocess = popen( c.c_str() ,"w");
                if(!m_subprocess){
                    ERRORMSG("Subprocess not opened!");
                }

            }else{
                //open file
                m_frameFile.open(p.string(),std::ios::trunc | std::ios::out);
                if(!m_frameFile.is_open()) {
                    ERRORMSG("Render frame at : " << p << " could not be opened!")
                }

                m_frameFile << "# Frame for t: " << GET_ISOCKET_REF_VALUE(Time) << "\n";
            }
        }
        void closeFile(){
            if(m_pipeToSubProcess){
                if(m_subprocess){
                    pclose(m_subprocess);
                    m_subprocess = nullptr;
                }
            }else{
                m_frameFile.close();
            }
        }

        bool isFileOpen(){
            if(m_pipeToSubProcess){
                return m_subprocess;
            }else{
                return m_frameFile.is_open();
            }
        }

        void dumpStream(){
            if(m_pipeToSubProcess){
                // Write to subprocess
                fputs(m_s.str().c_str(),m_subprocess);
            }
            else{
                // Write to file
                m_frameFile << m_s.rdbuf();
                //ASSERTMSG(m_frameFile.good(),"FUCK"); // stream will fail because s can contain nothing, therefore clear()
                m_frameFile.clear();
            }
            m_s.clear();
            m_s.str("");
            m_s.seekp(0);
            m_s.seekg(0);
        }

        GeometryMapType * m_geomMap = nullptr;

        std::stringstream m_s;
        std::fstream m_frameFile;

        bool m_pipeToSubProcess;
        std::string m_command;
        FILE * m_subprocess =nullptr;

        RendermanGeometryWriter m_geomWriter;


        // Homogenous Transform
        // y = R * x + t = [ R , t ;
        //                  0 , 1 ] * [x ; 1]
        // Renderman uses left multiplying homogenous matrices to transform points:
        // That means (' means transpose):
        // y' =  [x' , 1] * [ R1' , 0 ;  *  [ R2' , 0 ; = R2' * (x' * R1' +  t1')  + t2'
        //                    t1' , 1 ]       t2' , 1 ]
        //
        // Scale 1 1 -1
        // Translate 0 0 -3
        // Rotate -45 1 0 0
        // ===>   [x',1] * scale(1,1,-1) * translate(0,0,-3) * rotate(-45, 1,0,0) = [y',1]

        // Renderman stores matrices in row-major order!

        void writePosQuat(std::stringstream & s) {
            static Matrix44 T = Matrix44::Identity();

            T.block<3,3>(0,0) = GET_ISOCKET_REF_VALUE(BodyOrientation).matrix();
            T.row(3).head<3>() = GET_ISOCKET_REF_VALUE(BodyPosition);
              s << "ConcatTransform ["

              << T(0,0) << " "
              << T(0,1) << " "
              << T(0,2) << " "
              << T(0,3) << " "

              << T(1,0) << " "
              << T(1,1) << " "
              << T(1,2) << " "
              << T(1,3) << " "

              << T(2,0) << " "
              << T(2,1) << " "
              << T(2,2) << " "
              << T(2,3) << " "

              << T(3,0) << " "
              << T(3,1) << " "
              << T(3,2) << " "
              << T(3,3) << "]\n";
        };


    };

};



#endif
