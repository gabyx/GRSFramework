// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include "GRSF/common/OgreMeshExtraction.hpp"

// Raycast an Entitiy with Ray
// as found in http://test.ogitor.org/tiki/Raycasting%20to%20the%20polygon%20level
std::pair<bool, Ogre::Vector3> RaycastEntity(Ogre::Ray& ray, Ogre::Entity* pentity)
{
    std::pair<bool, Ogre::Vector3> results;

    // mesh data to retrieve
    size_t         vertex_count;
    size_t         index_count;
    Ogre::Vector3* vertices;
    unsigned long* indices;

    // Get the mesh information
    getMeshInformation(pentity, vertex_count, vertices, index_count, indices);

    // Transform Ray to Coordinate Frame of Entitiy
    // ray.setOrigin(pentity->getParentNode()->_getDerivedOrientation().Inverse() * (ray.getOrigin() -
    // pentity->getParentNode()->_getDerivedPosition() ) );
    ray.setOrigin(pentity->getParentNode()->_getFullTransform().inverse() * (ray.getOrigin()));
    ray.setDirection(pentity->getParentNode()->_getDerivedOrientation().Inverse() * ray.getDirection());

    // Output all Vertices/Indices
    //    for(int i =0; i<vertex_count; i++){
    //    cout << vertices[i] << endl;
    //    }
    //
    //    for(int i =0; i<index_count; i+=3){
    //        cout << indices[i]<<","<<indices[i+1]<<","<<indices[i+2]<< endl;
    //    }

    // Test for hitting individual triangles on the mesh
    bool       closest_found    = false;
    Ogre::Real closest_distance = -1.0;

    for (int i = 0; i < static_cast<int>(index_count); i += 3)
    {
        // Check for a hit against this triangle
        std::pair<bool, Ogre::Real> hit = Ogre::Math::intersects(
            ray, vertices[indices[i]], vertices[indices[i + 1]], vertices[indices[i + 2]], true, false);
        // If it was a hit check if its the closest
        if (hit.first)
        {
            if ((closest_distance < 0.0f) || (hit.second < closest_distance))
            {
                // this is the closest so far, save it off
                closest_distance = hit.second;
                closest_found    = true;
            }
        }
    }

    // Free the verticies and indicies memory
    delete[] vertices;
    delete[] indices;

    // If we found a  closest raycast for this object, update the
    // closest_result before moving on (to the next object)
    if (closest_found)
    {
        results.first  = true;
        results.second = ray.getPoint(closest_distance);
    }
    else
    {
        results.first  = false;
        results.second = Ogre::Vector3(0, 0, 0);
    }

    return results;
};

// Get the mesh information for the given mesh.
// Code found in Wiki: www.ogre3d.org/wiki/index.php/RetrieveVertexData
// All Mesh data is resolved in the Parent Frame of the Entitiy
void getMeshInformation(
    Ogre::Entity* entity, size_t& vertex_count, Ogre::Vector3*& vertices, size_t& index_count, unsigned long*& indices)
{
    bool   added_shared   = false;
    size_t current_offset = 0;
    size_t shared_offset  = 0;
    size_t next_offset    = 0;
    size_t index_offset   = 0;
    vertex_count = index_count = 0;

    Ogre::MeshPtr mesh = entity->getMesh();

    bool useSoftwareBlendingVertices = entity->hasSkeleton();

    if (useSoftwareBlendingVertices)
    {
        entity->_updateAnimation();
    }

    // Calculate how many vertices and indices we're going to need
    for (unsigned short i = 0; i < mesh->getNumSubMeshes(); ++i)
    {
        Ogre::SubMesh* submesh = mesh->getSubMesh(i);

        // We only need to add the shared vertices once
        if (submesh->useSharedVertices)
        {
            if (!added_shared)
            {
                vertex_count += mesh->sharedVertexData->vertexCount;
                added_shared = true;
            }
        }
        else
        {
            vertex_count += submesh->vertexData->vertexCount;
        }

        // Add the indices
        index_count += submesh->indexData->indexCount;
    }

    // Allocate space for the vertices and indices
    vertices = new Ogre::Vector3[vertex_count];
    indices  = new unsigned long[index_count];

    added_shared = false;

    // Run through the submeshes again, adding the data into the arrays
    for (unsigned short i = 0; i < mesh->getNumSubMeshes(); ++i)
    {
        Ogre::SubMesh* submesh = mesh->getSubMesh(i);

        //----------------------------------------------------------------
        // GET VERTEXDATA
        //----------------------------------------------------------------

        // Ogre::VertexData* vertex_data = submesh->useSharedVertices ? mesh->sharedVertexData : submesh->vertexData;
        Ogre::VertexData* vertex_data;

        // When there is animation:
        if (useSoftwareBlendingVertices)
        {
#ifdef BUILD_AGAINST_AZATHOTH
            vertex_data = submesh->useSharedVertices ? entity->_getSharedBlendedVertexData()
                                                     : entity->getSubEntity(i)->_getBlendedVertexData();
#else
            vertex_data = submesh->useSharedVertices ? entity->_getSkelAnimVertexData()
                                                     : entity->getSubEntity(i)->_getSkelAnimVertexData();
#endif
        }
        else
        {
            vertex_data = submesh->useSharedVertices ? mesh->sharedVertexData : submesh->vertexData;
        }

        if ((!submesh->useSharedVertices) || (submesh->useSharedVertices && !added_shared))
        {
            if (submesh->useSharedVertices)
            {
                added_shared  = true;
                shared_offset = current_offset;
            }

            const Ogre::VertexElement* posElem =
                vertex_data->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);

            Ogre::HardwareVertexBufferSharedPtr vbuf =
                vertex_data->vertexBufferBinding->getBuffer(posElem->getSource());

            unsigned char* vertex = static_cast<unsigned char*>(vbuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

            // There is _no_ baseVertexPointerToElement() which takes an Ogre::Real or a double
            //  as second argument. So make it float, to avoid trouble when Ogre::Real will
            //  be comiled/typedefed as double:
            //      Ogre::Real* pReal;
            float* pReal;

            for (size_t j = 0; j < vertex_data->vertexCount; ++j, vertex += vbuf->getVertexSize())
            {
                posElem->baseVertexPointerToElement(vertex, &pReal);

                Ogre::Vector3 pt(pReal[0], pReal[1], pReal[2]);

                vertices[current_offset + j] = pt;  //(orient * (pt * scale)) + position;
            }

            vbuf->unlock();
            next_offset += vertex_data->vertexCount;
        }

        Ogre::IndexData*                   index_data = submesh->indexData;
        size_t                             numTris    = index_data->indexCount / 3;
        Ogre::HardwareIndexBufferSharedPtr ibuf       = index_data->indexBuffer;

        bool use32bitindexes = (ibuf->getType() == Ogre::HardwareIndexBuffer::IT_32BIT);

        unsigned long*  pLong  = static_cast<unsigned long*>(ibuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
        unsigned short* pShort = reinterpret_cast<unsigned short*>(pLong);

        size_t offset      = (submesh->useSharedVertices) ? shared_offset : current_offset;
        size_t index_start = index_data->indexStart;
        size_t last_index  = numTris * 3 + index_start;

        if (use32bitindexes)
            for (size_t k = index_start; k < last_index; ++k)
            {
                indices[index_offset++] = pLong[k] + static_cast<unsigned long>(offset);
            }

        else
            for (size_t k = index_start; k < last_index; ++k)
            {
                indices[index_offset++] = static_cast<unsigned long>(pShort[k]) + static_cast<unsigned long>(offset);
            }

        ibuf->unlock();
        current_offset = next_offset;
    }
};

// Get the mesh information for the given mesh.
// Code found in Wiki: www.ogre3d.org/wiki/index.php/RetrieveVertexData
// All Mesh data is turned and orientated with position, orient, scale!
void getMeshInformation(Ogre::Entity*        entity,
                        size_t&              vertex_count,
                        Ogre::Vector3*&      vertices,
                        size_t&              index_count,
                        unsigned long*&      indices,
                        const Ogre::Matrix4& A_IK)
{
    bool   added_shared   = false;
    size_t current_offset = 0;
    size_t shared_offset  = 0;
    size_t next_offset    = 0;
    size_t index_offset   = 0;
    vertex_count = index_count = 0;

    Ogre::MeshPtr mesh = entity->getMesh();

    bool useSoftwareBlendingVertices = entity->hasSkeleton();

    if (useSoftwareBlendingVertices)
    {
        entity->_updateAnimation();
    }

    // Calculate how many vertices and indices we're going to need
    for (unsigned short i = 0; i < mesh->getNumSubMeshes(); ++i)
    {
        Ogre::SubMesh* submesh = mesh->getSubMesh(i);

        // We only need to add the shared vertices once
        if (submesh->useSharedVertices)
        {
            if (!added_shared)
            {
                vertex_count += mesh->sharedVertexData->vertexCount;
                added_shared = true;
            }
        }
        else
        {
            vertex_count += submesh->vertexData->vertexCount;
        }

        // Add the indices
        index_count += submesh->indexData->indexCount;
    }

    // Allocate space for the vertices and indices
    vertices = new Ogre::Vector3[vertex_count];
    indices  = new unsigned long[index_count];

    added_shared = false;

    // Run through the submeshes again, adding the data into the arrays
    for (unsigned short i = 0; i < mesh->getNumSubMeshes(); ++i)
    {
        Ogre::SubMesh* submesh = mesh->getSubMesh(i);

        //----------------------------------------------------------------
        // GET VERTEXDATA
        //----------------------------------------------------------------

        // Ogre::VertexData* vertex_data = submesh->useSharedVertices ? mesh->sharedVertexData : submesh->vertexData;
        Ogre::VertexData* vertex_data;

        // When there is animation:
        if (useSoftwareBlendingVertices)
        {
#ifdef BUILD_AGAINST_AZATHOTH
            vertex_data = submesh->useSharedVertices ? entity->_getSharedBlendedVertexData()
                                                     : entity->getSubEntity(i)->_getBlendedVertexData();
#else
            vertex_data = submesh->useSharedVertices ? entity->_getSkelAnimVertexData()
                                                     : entity->getSubEntity(i)->_getSkelAnimVertexData();
#endif
        }
        else
        {
            vertex_data = submesh->useSharedVertices ? mesh->sharedVertexData : submesh->vertexData;
        }

        if ((!submesh->useSharedVertices) || (submesh->useSharedVertices && !added_shared))
        {
            if (submesh->useSharedVertices)
            {
                added_shared  = true;
                shared_offset = current_offset;
            }

            const Ogre::VertexElement* posElem =
                vertex_data->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);

            Ogre::HardwareVertexBufferSharedPtr vbuf =
                vertex_data->vertexBufferBinding->getBuffer(posElem->getSource());

            unsigned char* vertex = static_cast<unsigned char*>(vbuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

            // There is _no_ baseVertexPointerToElement() which takes an Ogre::Real or a double
            //  as second argument. So make it float, to avoid trouble when Ogre::Real will
            //  be comiled/typedefed as double:
            //      Ogre::Real* pReal;
            float* pReal;

            for (size_t j = 0; j < vertex_data->vertexCount; ++j, vertex += vbuf->getVertexSize())
            {
                posElem->baseVertexPointerToElement(vertex, &pReal);

                Ogre::Vector3 pt(pReal[0], pReal[1], pReal[2]);

                vertices[current_offset + j] = (A_IK * pt);
            }

            vbuf->unlock();
            next_offset += vertex_data->vertexCount;
        }

        Ogre::IndexData*                   index_data = submesh->indexData;
        size_t                             numTris    = index_data->indexCount / 3;
        Ogre::HardwareIndexBufferSharedPtr ibuf       = index_data->indexBuffer;

        bool use32bitindexes = (ibuf->getType() == Ogre::HardwareIndexBuffer::IT_32BIT);

        unsigned long*  pLong  = static_cast<unsigned long*>(ibuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
        unsigned short* pShort = reinterpret_cast<unsigned short*>(pLong);

        size_t offset      = (submesh->useSharedVertices) ? shared_offset : current_offset;
        size_t index_start = index_data->indexStart;
        size_t last_index  = numTris * 3 + index_start;

        if (use32bitindexes)
            for (size_t k = index_start; k < last_index; ++k)
            {
                indices[index_offset++] = pLong[k] + static_cast<unsigned long>(offset);
            }

        else
            for (size_t k = index_start; k < last_index; ++k)
            {
                indices[index_offset++] = static_cast<unsigned long>(pShort[k]) + static_cast<unsigned long>(offset);
            }

        ibuf->unlock();
        current_offset = next_offset;
    }
};
