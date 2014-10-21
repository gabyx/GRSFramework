#ifndef  RenderOutputLogic_hpp
#define  RenderOutputLogic_hpp

#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include "LogicNode.hpp"
#include "LookUpTable.hpp"

#include "RendermanGeometryWriter.hpp"
#include "RenderData.hpp"
#include "QuaternionHelpers.hpp"

namespace LogicNodes {



class RenderOutput : public LogicNode {
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

    struct Inputs {
        enum {
            BodyId,
            BodyPosition,
            BodyOrientation,
            Material,
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
    DECLARE_ISOCKET_TYPE(Material,  RenderMaterialPtr);
    DECLARE_ISOCKET_TYPE(BodyPosition, Vector3 );
    DECLARE_ISOCKET_TYPE(BodyOrientation, Quaternion );


    RenderOutput(unsigned int id) : LogicNode(id) {
        ADD_ISOCK(BodyId,0);
        ADD_ISOCK(BodyPosition,Vector3(0,0,0));
        ADD_ISOCK(BodyOrientation,Quaternion(1,0,0,0));
        ADD_ISOCK(Material,nullptr);
    }

    virtual void initFrame(boost::filesystem::path folder, std::string name, double time){};

    virtual ~RenderOutput() {};
    virtual void compute() {};

protected:
    boost::filesystem::path m_folder;
    std::string m_name;
};


class RendermanOutput : public RenderOutput {
public:
    DEFINE_RENDERCONVERTERDATA_CONFIG_TYPES
    using GeometryMapType = typename RenderDataType::GeometryMapType;

    RendermanOutput(unsigned int id, GeometryMapType * geomMap)
        : RenderOutput(id), m_geomMap(geomMap) {
        m_folder = "./";
        m_name = "Frame";
    }

    virtual ~RendermanOutput() {
        m_frameFile.close();
    }

    void compute() {

        // Write the begin block of the rib file
        std::stringstream s;
        s << "AttributeBegin\n";

        // Write position and quaternion
        writePosQuat(s);

        // First get the id of the body then
        RigidBodyIdType & id = GET_ISOCKET_REF_VALUE(BodyId);
        auto geomIt = m_geomMap->find( id );
        if( geomIt == m_geomMap->end() ) {
            ERRORMSG("Geometry for body id: " << id << " not found!")
        }

        // Write the geometry
        m_geomWriter.setStream(&s);
        geomIt->second.apply_visitor(m_geomWriter);

        // Write the material
        GET_ISOCKET_REF_VALUE(Material)->write(s);

        // finish
        s << "AttributeEnd\n";

        // Write to file
        m_frameFile << s.rdbuf();
    }

    void initFrame(boost::filesystem::path folder, std::string filename, double time) {
        m_folder = folder;
        m_name = filename;

        // open new RIB file at this path and name
        m_frameFile.close();

        boost::filesystem::path file = m_folder / (m_name + ".rib");
        m_frameFile.open(file.string(),std::ios::trunc | std::ios::out);

        if(!m_frameFile.is_open()) {
            ERRORMSG("Render frame at : " << file << " could not be opened!")
        }

        m_frameFile << "# Frame for t: " << time << "\n";
    }

protected:
    GeometryMapType * m_geomMap;
    std::fstream m_frameFile;

    RendermanGeometryWriter m_geomWriter;

    void writePosQuat(std::stringstream & s) {
        static Matrix44 T = Matrix44::Identity();

        QuaternionHelpers::setRotFromQuaternion(GET_ISOCKET_REF_VALUE(BodyOrientation), T.block<3,3>(0,0) );
        T.col(3).head<3>() = GET_ISOCKET_REF_VALUE(BodyPosition);
        s << "Transform ["
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

