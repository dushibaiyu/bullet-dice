/**
 * Copyright (c) 2011-2014 Microsoft Mobile and/or its subsidiary(-ies).
 * All rights reserved.
 *
 * OpenGL ES 2.0 and physics example.
 *
 * For the applicable distribution terms see the license.txt -file, included in
 * the distribution.
 */

#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "Mesh.h"

using namespace ExampleLoader;



Mesh::Mesh() {
    m_name = 0;
    m_materialName = 0;
    nextMesh = 0;
    m_vertices = 0;
    m_vertexCount = 0;
    m_indices = 0;
    m_indexCount = 0;
}


Mesh::~Mesh()
{
    setName(0);
    setMaterialName(0);
    release();
}


Mesh* ExampleLoader::loadDig3D(const char *s, int len, float loadScale)
{
    int pos = 0;
    int f;
    char loadTemp[512];
    memcpy(loadTemp, s + pos, 6 );
    pos += 6;
    if (strcmp(loadTemp, "DIG3D") != 0)
        return 0;	    // incorrect header

    Mesh *rval = 0;
    while (1) {             // loop to load the meshes.
        f=0;
        while (pos < len) {
            loadTemp[f] = s[pos];
            pos++;
            if (loadTemp[f]==0)
                break;
            f++;
        }
        if (pos >= len)
            return rval;

        Mesh *nmesh = new Mesh;
        nmesh->nextMesh = rval;
        rval = nmesh;
        nmesh->setName(loadTemp);

        f=0;
        while (pos<len) {
            loadTemp[f] = s[pos];
            pos++;
            if (loadTemp[f]==0)
                break;
            f++;
        }
        nmesh->setMaterialName(loadTemp);

        // scaling information
        float pscale[4];
        memcpy(pscale, s + pos, sizeof(float) * 4);
        pos += sizeof(float) * 4;

        int vertexCount, faceCount;
        memcpy(&vertexCount, s + pos, sizeof(int));
        pos += sizeof(int);
        memcpy(&faceCount, s + pos, sizeof(int));
        pos += sizeof(int);
        unsigned char has_normals = (unsigned char)s[pos];
        pos++;
        nmesh->reinit(vertexCount, faceCount);

        ExampleLoader::Vertex *ver = nmesh->getVertices();
        unsigned short rtemp[3];
        unsigned int itemp;

        // read vertices
        for (f=0; f<vertexCount; f++) {
            memcpy(rtemp, s + pos, sizeof(unsigned short) * 3);
            pos += sizeof(unsigned short) * 3;
            ver[f].coord[0] = (float)rtemp[0] / pscale[3] + pscale[0];
            ver[f].coord[1] = (float)rtemp[1] / pscale[3] + pscale[1];
            ver[f].coord[2] = (float)rtemp[2] / pscale[3] + pscale[2];
            ver[f].coord[0] *= loadScale;
            ver[f].coord[1] *= loadScale;
            ver[f].coord[2] *= loadScale;

            memcpy(&itemp, s+pos, sizeof(unsigned int));
            pos += sizeof(unsigned int);
            ver[f].uv[0] = (((float)(itemp & 65535)) - 32768.0f) / 4096.0f;
            ver[f].uv[1] = ((((float)((itemp >> 16) & 65535))) - 32768.0f) / 4096.0f;

            if (has_normals) {
                memcpy(&itemp, s + pos, sizeof(unsigned int));
                pos += sizeof(unsigned int);
                ver[f].normal[0] = (((float)(itemp & 1023)) - 512.0f) / 511.0f;
                ver[f].normal[1] = (((float)((itemp>>10) & 1023))-512.0f) / 511.0f;
                ver[f].normal[2] = (((float)((itemp>>20) & 1023))-512.0f) / 511.0f;
            }
            else {
                ver[f].normal[0] = 0.0f;
                ver[f].normal[1] = 0.0f;
                ver[f].normal[2] = 0.0f;
            }
        }

        unsigned short *indices = nmesh->getIndices();
        memcpy(indices, s+pos, faceCount * 3 * sizeof(unsigned short));
        pos += faceCount * 3 * sizeof(unsigned short);

        if (has_normals == 0) {
            nmesh->calculateNormals();
        }
    }

    return rval;
}



Mesh* ExampleLoader::loadDig3D( FILE *from, float loadScale ) {
    int f;
    char loadTemp[512];
    fread(loadTemp, 6, 1, from);
    if (strcmp( loadTemp, "DIG3D") !=0)
        return 0;           // incorrect header

    Mesh *rval = 0;
    while (1) {             // loop to load the meshes.
        f=0;
        while(1) {
            if (fread(&loadTemp[f],1,1, from) != 1)
                return rval;
            if (loadTemp[f] == 0)
                break;
            f++;
        }

        Mesh *nmesh = new Mesh;
        nmesh->nextMesh = rval;
        rval = nmesh;
        nmesh->setName( loadTemp );
        f = 0;
        while(1) {
            if (fread(&loadTemp[f],1,1, from) != 1)
                return rval;
            if (loadTemp[f]==0)
                break;
            f++;
        }
        nmesh->setMaterialName(loadTemp);

        // Scaling information
        float pscale[4];
        fread(pscale, sizeof(float), 4, from);

        int vertexCount, faceCount;
        fread(&vertexCount, sizeof( int ) , 1, from);
        fread( &faceCount, sizeof( int ) , 1, from);
        unsigned char has_normals = 0;
        fread( &has_normals, sizeof(unsigned char), 1, from);
        nmesh->reinit( vertexCount, faceCount );

        ExampleLoader::Vertex *ver = nmesh->getVertices();
        unsigned short rtemp[3];
        unsigned int itemp;

        // read vertices
        for (f=0; f<vertexCount; f++) {
            fread(rtemp, sizeof( unsigned short), 3, from);
            ver[f].coord[0] = (float)rtemp[0] / pscale[3] + pscale[0];
            ver[f].coord[1] = (float)rtemp[1] / pscale[3] + pscale[1];
            ver[f].coord[2] = (float)rtemp[2] / pscale[3] + pscale[2];

            ver[f].coord[0] *= loadScale;
            ver[f].coord[1] *= loadScale;
            ver[f].coord[2] *= loadScale;

            fread(&itemp, sizeof(unsigned int), 1, from);	    // uv
            ver[f].uv[0] = (((float)(itemp&65535))-32768.0f) / 4096.0f;
            ver[f].uv[1] = ((((float)((itemp>>16)&65535)))-32768.0f) / 4096.0f;
            if (has_normals) {
                fread( &itemp, sizeof(unsigned int), 1, from);	    // normal
                ver[f].normal[0] = (((float)(itemp&1023))-512.0f) / 511.0f;
                ver[f].normal[1] = (((float)((itemp>>10)&1023))-512.0f) / 511.0f;
                ver[f].normal[2] = (((float)((itemp>>20)&1023))-512.0f) / 511.0f;
            }
            else {
                ver[f].normal[0] = 0.0f;
                ver[f].normal[1] = 0.0f;
                ver[f].normal[2] = 0.0f;
            }
        }

        unsigned short *indices = nmesh->getIndices();
        fread(indices, sizeof(unsigned short), faceCount * 3, from);

        if (has_normals == 0) {
            nmesh->calculateNormals();
        }
    }

    return rval;
}


void Mesh::setName(const char *name) {
    if (m_name) {
        delete [] m_name;
        m_name = 0;
    }

    if (!name)
        return;

    int le = strlen(name);
    m_name = new char[le+1];
    strcpy(m_name, name);
}


void Mesh::setMaterialName(const char *name)
{
    if (m_materialName) {
        delete [] m_materialName;
        m_materialName = 0;
    }

    if (!name)
        return;

    int le = strlen(name);
    m_materialName = new char[le + 1];
    strcpy(m_materialName, name);
}


void Mesh::release()
{
    if (m_vertices)
        delete [] m_vertices;

    m_vertices = 0;
    if (m_indices)
        delete [] m_indices;

    m_indices = 0;
    m_vertexCount =0;
    m_indexCount = 0;
}


void Mesh::reinit(int vertexCount, int faceCount)
{
    release();
    m_vertexCount = vertexCount;
    m_vertices = new Vertex[m_vertexCount];
    m_indexCount = faceCount * 3;
    m_indices = new unsigned short[m_indexCount];
}


void Mesh::calculateNormals()
{
    if(m_vertexCount < 1 || m_indexCount < 3)
        return;

    int f,g;
    for (f=0; f<m_vertexCount; f++) {
        m_vertices->normal[0] = 0.0f;
        m_vertices->normal[1] = 0.0f;
        m_vertices->normal[2] = 0.0f;
    }

    int p1i,p2i,p3i;
    float v1[3], v2[3], nor[3];
    float l;

    for (f=0; f<m_indexCount/3; f++) {
        p1i = m_indices[f * 3 + 0];
        p2i = m_indices[f * 3 + 1];
        p3i = m_indices[f * 3 + 2];

        for (g=0; g<3; g++){
            v1[g] = m_vertices[p2i].coord[g] - m_vertices[p1i].coord[g];
            v2[g] = m_vertices[p3i].coord[g] - m_vertices[p1i].coord[g];
        }

        l = sqrtf(v1[0]*v1[0] + v1[1]*v1[1] + v1[2]*v1[2]);
        v1[0] /= l;
        v1[1] /= l;
        v1[2] /= l;

        l = sqrtf(v2[0]*v2[0] + v2[1]*v2[1] + v2[2]*v2[2]);
        v2[0] /= l;
        v2[1] /= l;
        v2[2] /= l;

        nor[0] = v1[1]*v2[2] - v1[2]*v2[1];
        nor[1] = v1[2]*v2[0] - v1[0]*v2[2];
        nor[2] = v1[0]*v2[1] - v1[1]*v2[0];
        l = sqrtf(nor[0]*nor[0] + nor[1]*nor[1] + nor[2]*nor[2]);
        nor[0] /= l;
        nor[1] /= l;
        nor[2] /= l;

        // Spread the faceNormal
        for (g=0; g<3; g++) {
            m_vertices[p1i].normal[g] += nor[g];
            m_vertices[p2i].normal[g] += nor[g];
            m_vertices[p3i].normal[g] += nor[g];
        }
    }

    for (f=0; f<m_vertexCount; f++) {
        l = sqrtf( m_vertices[f].normal[0]*m_vertices[f].normal[0] +
                   m_vertices[f].normal[1]*m_vertices[f].normal[1] +
                   m_vertices[f].normal[2]*m_vertices[f].normal[2]);

        m_vertices[f].normal[0] /= l;
        m_vertices[f].normal[1] /= l;
        m_vertices[f].normal[2] /= l;
    }
}
