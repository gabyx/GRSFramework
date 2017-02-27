// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef MeshGeometry_hpp
#define MeshGeometry_hpp

#include <boost/serialization/access.hpp>
#include <boost/shared_ptr.hpp>

#include "Asserts.hpp"
#include "ConfigureFile.hpp"

#if USE_OPCODE == 1
#include <Opcode.h>
#else
#if USE_OZCOLLIDE == 1
#define OZCOLLIDE_PCH
#include <ozcollide/ozcollide.h>
#endif
#endif

#include "TypeDefs.hpp"
#include "TypeDefs.hpp"

#include "MeshData.hpp"

template <class PREC>
class MeshGeometry
{
public:
    DEFINE_MATRIX_TYPES
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MeshGeometry()
    {
        m_pMeshData = NULL;
    }

    MeshGeometry(MeshData<MeshPREC>* pMeshData) : m_pMeshData(pMeshData)
    {
#if USE_OPCODE == 1

// ASSERTMSG(sizeof(MeshPREC) == sizeof(OPCODE_PRECISION), " ATTENTION: OPCODE seems to use another precision than the
// MeshData!");
// Dependent on the use of our collision library, build here related stuff...

//      Opcode::MeshInterface* meshInterface = new Opcode::MeshInterface();
//	   meshInterface->SetNbTriangles((unsigned int)m_pMeshData->m_Faces.size());
//	   meshInterface->SetNbVertices((unsigned int)m_pMeshData->m_Vertices.size());
//	   meshInterface->SetPointers((const IceMaths::IndexedTriangle*)(m_pMeshData->m_Faces[0].data()), (const
// IceMaths::Point*)(m_pMeshData->m_Vertices[0].data()));
//
//      Opcode::OPCODECREATE create;
//	   create.mIMesh = meshInterface;
//	   create.mSettings.mLimit	= 1;
//	   create.mSettings.mRules	= Opcode::SPLIT_SPLATTER_POINTS | Opcode::SPLIT_GEOM_CENTER;
//	   create.mNoLeaf			= true;
//	   create.mQuantized		= true;
//	   create.mKeepOriginal	= false;
//	   create.mCanRemap		= false;
//
//	   m_pOpcodeModel = new Opcode::Model();
//	   m_pOpcodeModel->Build(create);
#endif

#if USE_OZCOLLIDE == 1
        using namespace ozcollide;
        AABBTreePolyBuilder builder;

        if (pMeshData->m_Vertices.size() > std::numeric_limits<int>::max())
        {
            GRSF_ERRORMSG("The mesh has vertice indices which are to big to put into an INT in ozcollide");
        }
        if (pMeshData->m_Faces.size() > std::numeric_limits<int>::max())
        {
            GRSF_ERRORMSG("The mesh has to many faces to put into an INT in ozcollide");
        }

        m_ozPolys.reserve(pMeshData->m_Faces.size());
        std::vector<int> a;
        for (int i = 0; i < pMeshData->m_Faces.size(); i++)
        {
            // poly.setIndicesMemory(3,reinterpret_cast<int *>(pMeshData->m_Faces[i].data() ));
            ozcollide::Polygon poly;
            // poly.setNbIndices(3);
            std::cout << pMeshData->m_Faces[i] << "," << std::endl;
            std::cout << sizeof(int) << std::endl;
            poly.setIndex(0, (int)pMeshData->m_Faces[i](0));
            poly.setIndex(1, (int)pMeshData->m_Faces[i](1));
            poly.setIndex(2, (int)pMeshData->m_Faces[i](2));

            std::cout << poly.getIndex(0) << "," << poly.getIndex(1) << "," << poly.getIndex(2) << std::endl;
            m_ozPolys.push_back(poly);  ///< ATTTENTION THIS DOES NOT WORK!!!
            a.push_back(i);
            // std::cout <<  m_ozPolys.back().getIndex(0) <<","<< m_ozPolys.back().getIndex(1)   <<","<<
            // m_ozPolys.back().getIndex(2)  <<std::endl;

            std::cout << "=================" << std::endl;

            ozcollide::Polygon* pols = (&m_ozPolys[0]);
            for (unsigned int l = 0; l < m_ozPolys.size(); l++)
            {
                ozcollide::Polygon& pol = m_ozPolys[l];
                std::cout << a[l] << ": " << pol.getIndex(0) << "," << pol.getIndex(1) << "," << pol.getIndex(2)
                          << std::endl;
            }
            std::cout << "=================" << std::endl;
        }

        std::cout << "=================" << std::endl;
        for (unsigned int l = 0; l < m_ozPolys.size(); l++)
        {
            ozcollide::Polygon& pol = m_ozPolys[l];
            std::cout << pol.getIndex(0) << "," << pol.getIndex(1) << "," << pol.getIndex(2) << std::endl;
        }
        std::cout << "=================" << std::endl;

        m_pTreePoly = builder.buildFromPolys(&m_ozPolys[0],
                                             m_ozPolys.size(),
                                             (const Vec3f*)m_pMeshData->m_Vertices[0].data(),
                                             m_pMeshData->m_Vertices.size(),
                                             1,
                                             NULL);
#endif
    };

    ~MeshGeometry()
    {
#if USE_OPCODE == 1
        delete m_pOpcodeModel->GetMeshInterface();
        delete m_pOpcodeModel;
#endif
#if USE_OZCOLLIDE == 1

#endif
        delete m_pMeshData;
    }

    MeshData<MeshPREC>* m_pMeshData;

#if USE_OPCODE == 1
    Opcode::Model* m_pOpcodeModel;
#endif

#if USE_OZCOLLIDE == 1
    ozcollide::AABBTreePoly*        m_pTreePoly;
    std::vector<ozcollide::Polygon> m_ozPolys;  ///< A set of polygons used in ozcollide, only referenced!
#endif

private:
    friend class boost::serialization::access;
};

#endif
