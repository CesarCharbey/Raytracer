#ifndef MESH_H
#define MESH_H

#include <vector>
#include <string>
#include <algorithm>
#include <cfloat>
#include <cmath>

#include "Vec3.h"
#include "Ray.h"
#include "Triangle.h"
#include "Material.h"

#include <GL/glut.h>

// structure représentant un sommet (position + normale + uv)
struct MeshVertex
{
    inline MeshVertex() {}
    inline MeshVertex(const Vec3 &_p, const Vec3 &_n) : position(_p), normal(_n), u(0), v(0) {}
    inline MeshVertex(const MeshVertex &vertex) : position(vertex.position), normal(vertex.normal), u(vertex.u), v(vertex.v) {}
    inline virtual ~MeshVertex() {}
    inline MeshVertex &operator=(const MeshVertex &vertex)
    {
        position = vertex.position;
        normal = vertex.normal;
        u = vertex.u;
        v = vertex.v;
        return (*this);
    }
    Vec3 position; // position
    Vec3 normal;   // normale
    float u, v;    // uv
};

// structure représentant un triangle via 3 indices de sommets
struct MeshTriangle
{
    inline MeshTriangle() { v[0] = v[1] = v[2] = 0; }
    inline MeshTriangle(const MeshTriangle &t)
    {
        v[0] = t.v[0];
        v[1] = t.v[1];
        v[2] = t.v[2];
    }
    inline MeshTriangle(unsigned int v0, unsigned int v1, unsigned int v2)
    {
        v[0] = v0;
        v[1] = v1;
        v[2] = v2;
    }
    unsigned int &operator[](unsigned int iv) { return v[iv]; }
    unsigned int operator[](unsigned int iv) const { return v[iv]; }
    inline virtual ~MeshTriangle() {}
    inline MeshTriangle &operator=(const MeshTriangle &t)
    {
        v[0] = t.v[0];
        v[1] = t.v[1];
        v[2] = t.v[2];
        return (*this);
    }
    unsigned int v[3];
};

class Mesh
{
protected:
    // conversion vertices -> tableau plat pour opengl
    void build_positions_array()
    {
        positions_array.resize(3 * vertices.size());
        for (unsigned int v = 0; v < vertices.size(); ++v)
        {
            positions_array[3 * v + 0] = vertices[v].position[0];
            positions_array[3 * v + 1] = vertices[v].position[1];
            positions_array[3 * v + 2] = vertices[v].position[2];
        }
    }
    // conversion normales -> tableau plat
    void build_normals_array()
    {
        normalsArray.resize(3 * vertices.size());
        for (unsigned int v = 0; v < vertices.size(); ++v)
        {
            normalsArray[3 * v + 0] = vertices[v].normal[0];
            normalsArray[3 * v + 1] = vertices[v].normal[1];
            normalsArray[3 * v + 2] = vertices[v].normal[2];
        }
    }
    // conversion uvs -> tableau plat
    void build_UVs_array()
    {
        uvs_array.resize(2 * vertices.size());
        for (unsigned int vert = 0; vert < vertices.size(); ++vert)
        {
            uvs_array[2 * vert + 0] = vertices[vert].u;
            uvs_array[2 * vert + 1] = vertices[vert].v;
        }
    }
    // conversion triangles -> tableau indices plat
    void build_triangles_array()
    {
        triangles_array.resize(3 * triangles.size());
        for (unsigned int t = 0; t < triangles.size(); ++t)
        {
            triangles_array[3 * t + 0] = triangles[t].v[0];
            triangles_array[3 * t + 1] = triangles[t].v[1];
            triangles_array[3 * t + 2] = triangles[t].v[2];
        }
    }

public:
    std::vector<MeshVertex> vertices;
    std::vector<MeshTriangle> triangles;

    // buffers opengl
    std::vector<float> positions_array;
    std::vector<float> normalsArray;
    std::vector<float> uvs_array;
    std::vector<unsigned int> triangles_array;

    Material material;

    void loadOFF(const std::string &filename);
    void recomputeNormals();
    void centerAndScaleToUnit();
    void scaleUnit();

    // KD interne par mesh
    // structure aabb (axis aligned bounding box)
    struct AABB
    {
        Vec3 bmin, bmax;
        AABB() : bmin(FLT_MAX, FLT_MAX, FLT_MAX),
                 bmax(-FLT_MAX, -FLT_MAX, -FLT_MAX) {}
        // agrandit la boite pour inclure le point p
        inline void expand(const Vec3 &p)
        {
            bmin[0] = std::min(bmin[0], p[0]);
            bmax[0] = std::max(bmax[0], p[0]);
            bmin[1] = std::min(bmin[1], p[1]);
            bmax[1] = std::max(bmax[1], p[1]);
            bmin[2] = std::min(bmin[2], p[2]);
            bmax[2] = std::max(bmax[2], p[2]);
        }
        // fusionne avec une autre boite
        inline void expand(const AABB &b)
        {
            expand(b.bmin);
            expand(b.bmax);
        }
        // retourne l'axe le plus étendu (0=x, 1=y, 2=z)
        inline int longestAxis() const
        {
            Vec3 e = bmax - bmin;
            if (e[0] >= e[1] && e[0] >= e[2])
                return 0;
            if (e[1] >= e[0] && e[1] >= e[2])
                return 1;
            return 2;
        }
        inline Vec3 centroid() const { return (bmin + bmax) * 0.5f; }
    };

    // structure accélératrice triangle (précalculs möller-trumbore)
    struct TriAccel
    {
        Vec3 p0, e1, e2;
        Vec3 Ng; // normale géo (pré-normalisée)
        AABB box;
        Vec3 center; // centroïde AABB
    };
    std::vector<TriAccel> tri_accel;

    // noeud de l'arbre kd interne
    struct KdNode
    {
        AABB box;
        int left = -1;
        int right = -1;
        int start = 0; // offset dans tri_index
        int count = 0; // >0 => feuille
        inline bool isLeaf() const { return count > 0; }
    };
    std::vector<KdNode> kd_nodes;
    std::vector<int> tri_index; // indices de triangles pour les feuilles

    // Construction des buffers + kd-tree
    virtual void build_arrays()
    {
        recomputeNormals();
        build_positions_array();
        build_normals_array();
        build_UVs_array();
        build_triangles_array();
        construire_tri_accel(); // précalcul géométrie
        construire_kdtree();    // construction hiérarchie
    }

    // Transfos
    // translation de tous les sommets
    void translate(Vec3 const &translation)
    {
        for (unsigned int v = 0; v < vertices.size(); ++v)
            vertices[v].position += translation;
    }

    // application matrice 3x3 générique
    void apply_transformation_matrix(Mat3 transform)
    {
        for (unsigned int v = 0; v < vertices.size(); ++v)
            vertices[v].position = transform * vertices[v].position;
    }

    // mise à l'échelle
    void scale(Vec3 const &scale)
    {
        Mat3 S(scale[0], 0., 0., 0., scale[1], 0., 0., 0., scale[2]);
        apply_transformation_matrix(S);
    }

    // rotation axe x
    void rotate_x(float angle)
    {
        float a = angle * M_PI / 180.f;
        Mat3 R(1., 0., 0., 0., cos(a), -sin(a), 0., sin(a), cos(a));
        apply_transformation_matrix(R);
    }

    // rotation axe y
    void rotate_y(float angle)
    {
        float a = angle * M_PI / 180.f;
        Mat3 R(cos(a), 0., sin(a), 0., 1., 0., -sin(a), 0., cos(a));
        apply_transformation_matrix(R);
    }

    // rotation axe z
    void rotate_z(float angle)
    {
        float a = angle * M_PI / 180.f;
        Mat3 R(cos(a), -sin(a), 0., sin(a), cos(a), 0., 0., 0., 1.);
        apply_transformation_matrix(R);
    }

    // Rendu GL
    void draw() const
    {
        if (triangles_array.empty())
            return;
        // configuration matériau opengl
        GLfloat material_color[4] = {material.diffuse_material[0], material.diffuse_material[1], material.diffuse_material[2], 1.0f};
        GLfloat material_specular[4] = {material.specular_material[0], material.specular_material[1], material.specular_material[2], 1.0f};
        GLfloat material_ambient[4] = {material.ambient_material[0], material.ambient_material[1], material.ambient_material[2], 1.0f};

        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, material_specular);
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, material_color);
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, material_ambient);
        glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, material.shininess);

        // envoi des tableaux de sommets et normales
        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_NORMAL_ARRAY);
        glNormalPointer(GL_FLOAT, 3 * sizeof(float), (GLvoid *)(normalsArray.data()));
        glVertexPointer(3, GL_FLOAT, 3 * sizeof(float), (GLvoid *)(positions_array.data()));
        glDrawElements(GL_TRIANGLES, triangles_array.size(), GL_UNSIGNED_INT, (GLvoid *)(triangles_array.data()));
    }

    // API d’intersection publique : utilise le KD interne
    RayTriangleIntersection intersect(Ray const &ray) const;
    bool anyHitBefore_kdtree(Ray const &ray, float tMax) const;

    // Construction KD interne (implémentées dans Mesh.cpp)
    void construire_tri_accel();
    void construire_kdtree();
    RayTriangleIntersection intersect_kdtree(Ray const &ray) const;

    // (utilitaire si besoin)
    static bool rayAABB(const Vec3 &O, const Vec3 &D, const Vec3 &invD, const int sgn[3],
                        const AABB &B, float &tmin, float &tmax);

    // Debug KD
    void debugDrawKd(int maxDepth = -1, bool leavesOnly = false) const;
};

#endif