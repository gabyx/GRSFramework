#ifndef GMSF_Dynamics_General_PrintGeometryDetails_hpp
#define GMSF_Dynamics_General_PrintGeometryDetails_hpp


#include <boost/variant.hpp>

#include "TypeDefs.hpp"

#include "AssertionDebug.hpp"
#include "LogDefines.hpp"

#include "SimpleLogger.hpp"

#include "SphereGeometry.hpp"
#include "BoxGeometry.hpp"
#include "HalfspaceGeometry.hpp"

class PrintGeometryDetailsVisitor : public boost::static_visitor<> {
    public:

        DEFINE_RIGIDBODY_CONFIG_TYPES

        PrintGeometryDetailsVisitor(Logging::Log * pLog, RigidBodyType::GeometryType & pGeom, std::string prefix):
            m_pLog(pLog),
            m_prefix(prefix)
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
            ASSERTMSG(false,"MeshGeometry PrintDetails: This has not been implemented yet!");
        }

        void operator()(std::shared_ptr<const HalfspaceGeometry > & halfspace)  {
            //This has not been implemented yet!
            ASSERTMSG(false,"HalfspaceGeometry PrintDetails: This has not been implemented yet!");
        }

        private:
        std::string m_prefix;
        Logging::Log  * m_pLog;

};





#endif
