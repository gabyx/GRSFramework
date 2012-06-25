
#ifndef MeshGeometry_hpp
#define MeshGeometry_hpp

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

#include "ConfigureFile.hpp"
#include "AssertionDebug.hpp"

#if USE_OPCODE == 1
#include <Opcode.h>
#else
#if USE_OZCOLLIDE == 1
#include <ozcollide/ozcollide.h>
#include <ozcollide/aabbtree.h>
#include <ozcollide/aabbtreepoly_builder.h>
#endif
#endif

#include "TypeDefs.hpp"

template<class PREC>
class MeshGeometry{
public:

  DEFINE_MATRIX_TYPES
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MeshGeometry(boost::shared_ptr<MeshData<MeshPREC> > & pMeshData):
  m_pMeshData(pMeshData)
  {

#if USE_OPCODE == 1

      ASSERTMSG(sizeof(MeshPREC) == sizeof(OPCODE_PRECISION), " ATTENTION: OPCODE seems to use another precision than the MeshData!");
      // Dependent on the use of our collision library, build here related stuff...



      Opcode::MeshInterface* meshInterface = new Opcode::MeshInterface();
	   meshInterface->SetNbTriangles((unsigned int)m_pMeshData->m_Faces.size());
	   meshInterface->SetNbVertices((unsigned int)m_pMeshData->m_Vertices.size());
	   meshInterface->SetPointers((const IceMaths::IndexedTriangle*)(m_pMeshData->m_Faces[0].data()), (const IceMaths::Point*)(m_pMeshData->m_Vertices[0].data()));

      Opcode::OPCODECREATE create;
	   create.mIMesh = meshInterface;
	   create.mSettings.mLimit	= 1;
	   create.mSettings.mRules	= Opcode::SPLIT_SPLATTER_POINTS | Opcode::SPLIT_GEOM_CENTER;
	   create.mNoLeaf			= true;
	   create.mQuantized		= true;
	   create.mKeepOriginal	= false;
	   create.mCanRemap		= false;

	   m_pOpcodeModel = new Opcode::Model();
	   m_pOpcodeModel->Build(create);
#endif

#if USE_OZCOLLIDE == 1
      using namespace ozcollide;
      AABBTreePolyBuilder builder;

      //m_pTreePoly = builder.buildFromPolys(,,(const Vec3f*)m_pMeshData->m_Vertices[0].data(),m_pMeshData->m_Vertices.size(),1,NULL)
#endif

  };

  ~MeshGeometry(){
#if USE_OPCODE == 1
    delete m_pOpcodeModel->GetMeshInterface();
    delete m_pOpcodeModel;
#endif
#if USE_OZCOLLIDE == 1

#endif
  }

  boost::shared_ptr<MeshData<MeshPREC> > m_pMeshData;

#if USE_OPCODE == 1
    Opcode::Model * m_pOpcodeModel;
#endif

#if USE_OZCOLLIDE == 1
    ozcollide::AABBTreePoly * m_pTreePoly;
#endif

private:

};


#endif
