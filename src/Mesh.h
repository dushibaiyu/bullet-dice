/**
 * Copyright (c) 2011 Nokia Corporation and/or its subsidiary(-ies).
 * All rights reserved.
 *
 * OpenGL ES 2.0 and physics example.
 *
 * For the applicable distribution terms see the license.txt -file, included in
 * the distribution.
 */

#ifndef CMESH_H
#define CMESH_H

#include <stdio.h>

namespace ExampleLoader {

struct Vertex {
    float coord[3];
    float normal[3];
    float uv[2];
};


class Mesh
{
public:
    Mesh();
    virtual ~Mesh();
    void reinit( int vertexCount, int faceCount );
    void release();

    void setName( const char *name );
    void setMaterialName( const char *matname );

    inline Vertex* getVertices() { return m_vertices; }
    inline unsigned short *getIndices() { return m_indices; }
    inline int getVertexCount() { return m_vertexCount; }
    inline int getIndexCount() { return m_indexCount; }
    inline int getFaceCount() { return m_indexCount/3; }

    inline char *getName() { return m_name; }
    inline char *getMaterialName() { return m_materialName; }

    void calculateNormals();

    Mesh *nextMesh;		    // for listing.


protected:
    Vertex *m_vertices;
    int m_vertexCount;

    // NOTE, n indexbuffers for N materials?
    unsigned short *m_indices;
    int m_indexCount;

    char *m_name;
    char *m_materialName;

};

Mesh* loadDig3D( FILE *from, float loadScale = 1.0f );		// returns a list of meshes from Dig3D
Mesh* loadDig3D( const char *s, int len, float loadScale = 1.0f );		// returns a list of meshes from Dig3D

}  // namespace ExampleLoader

#endif // CMESH_H
