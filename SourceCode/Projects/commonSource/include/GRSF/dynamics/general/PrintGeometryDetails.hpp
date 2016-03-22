#ifndef GRSF_Dynamics_General_PrintGeometryDetails_hpp
#define GRSF_Dynamics_General_PrintGeometryDetails_hpp


#include <boost/variant.hpp>

#include "GRSF/Common/TypeDefs.hpp"

#include "GRSF/Common/AssertionDebug.hpp"
#include "GRSF/Common/LogDefines.hpp"

#include "GRSF/Common/SimpleLogger.hpp"

#include "GRSF/Dynamics/Collision/Geometries.hpp"

class PrintGeometryDetailsVisitor : public boost::static_visitor<> {
    public:

        DEFINE_RIGIDBODY_CONFIG_TYPES

        PrintGeometryDetailsVisitor(Logging::Log * pLog, RigidBodyType::GeometryType & pGeom, std::string prefix):
            m_prefix(prefix),
            m_pLog(pLog)
        {
            boost::apply_visitor(*this, pGeom);
        }


        void operator()(std::shared_ptr<const SphereGeometry > & sphereGeom)  {
            LOG(m_pLog, m_prefix << "Sphere , radius: " <<sphereGeom->m_radius << std::endl);
        }

        void operator()(std::shared_ptr<const BoxGeometry > & box)  {
            LOG(m_pLog, m_prefix << "Box,  extent: " <<box->m_extent << std::endl);
        }

        void operator()(std::shared_ptr<const MeshGeometry > & mesh)  {
            ERRORMSG("MeshGeometry PrintDetails: This has not been implemented yet!");
        }
        void operator()(std::shared_ptr<const CapsuleGeometry > & mesh)  {
            ERRORMSG("CapsuleGeometry PrintDetails: This has not been implemented yet!");
        }

        void operator()(std::shared_ptr<const HalfspaceGeometry > & halfspace)  {
            //This has not been implemented yet!
            ERRORMSG("HalfspaceGeometry PrintDetails: This has not been implemented yet!");
        }

        private:
        std::string m_prefix;
        Logging::Log  * m_pLog;

};





#endif
