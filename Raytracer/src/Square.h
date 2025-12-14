#ifndef SQUARE_H
#define SQUARE_H
#include "Vec3.h"
#include <vector>
#include "Mesh.h"
#include <cmath>

struct RaySquareIntersection
{
    bool intersectionExists;
    float t;
    float u, v;
    Vec3 intersection;
    Vec3 normal;
};

class Square : public Mesh
{
public:
    Vec3 m_normal;
    Vec3 m_bottom_left;
    Vec3 m_right_vector;
    Vec3 m_up_vector;

    Square() : Mesh() {}
    Square(Vec3 const &bottomLeft, Vec3 const &rightVector, Vec3 const &upVector, float width = 1., float height = 1.,
           float uMin = 0.f, float uMax = 1.f, float vMin = 0.f, float vMax = 1.f) : Mesh()
    {
        setQuad(bottomLeft, rightVector, upVector, width, height, uMin, uMax, vMin, vMax);
    }

    void setQuad(Vec3 const &bottomLeft, Vec3 const &rightVector, Vec3 const &upVector, float width = 1., float height = 1.,
                 float uMin = 0.f, float uMax = 1.f, float vMin = 0.f, float vMax = 1.f)
    {
        m_right_vector = rightVector;
        m_up_vector = upVector;
        m_normal = Vec3::cross(rightVector, upVector);
        m_bottom_left = bottomLeft;

        m_normal.normalize();
        m_right_vector.normalize();
        m_up_vector.normalize();

        m_right_vector = m_right_vector * width;
        m_up_vector = m_up_vector * height;

        vertices.clear();
        vertices.resize(4);
        vertices[0].position = bottomLeft;
        vertices[0].u = uMin;
        vertices[0].v = vMin;
        vertices[1].position = bottomLeft + m_right_vector;
        vertices[1].u = uMax;
        vertices[1].v = vMin;
        vertices[2].position = bottomLeft + m_right_vector + m_up_vector;
        vertices[2].u = uMax;
        vertices[2].v = vMax;
        vertices[3].position = bottomLeft + m_up_vector;
        vertices[3].u = uMin;
        vertices[3].v = vMax;
        vertices[0].normal = vertices[1].normal = vertices[2].normal = vertices[3].normal = m_normal;
        triangles.clear();
        triangles.resize(2);
        triangles[0][0] = 0;
        triangles[0][1] = 1;
        triangles[0][2] = 2;
        triangles[1][0] = 0;
        triangles[1][1] = 2;
        triangles[1][2] = 3;
    }

    RaySquareIntersection intersect(const Ray &ray) const
    {
        RaySquareIntersection intersection;
        intersection.intersectionExists = false;

        Vec3 m_bottom_left = vertices[0].position;
        Vec3 m_right_vector = vertices[1].position - vertices[0].position;
        Vec3 m_up_vector = vertices[3].position - vertices[0].position;
        Vec3 m_normal = Vec3::cross(m_right_vector, m_up_vector);
        m_normal.normalize();
        // On force la normale à pointer vers l'intérieur du cube
        m_normal = -m_normal;

        Vec3 ray_dir = ray.direction();
        ray_dir.normalize();

        float denom = Vec3::dot(ray_dir, m_normal);
        if (denom < 1e-6f)
            return intersection; // rayon parallèle au plan

        float t = Vec3::dot(m_bottom_left - ray.origin(), m_normal) / denom;
        if (t < 1e-3f)
            return intersection; // intersection trop proche ou derrière

        // point d'intersection sur le plan
        Vec3 P = ray.origin() + t * ray_dir;
        Vec3 v = P - m_bottom_left;

        float width = m_right_vector.length();
        float height = m_up_vector.length();

        float proj1 = Vec3::dot(v, m_right_vector) / width;
        float proj2 = Vec3::dot(v, m_up_vector) / height;

        if (proj1 >= 0.f && proj1 <= width && proj2 >= 0.f && proj2 <= height)
        {
            intersection.intersectionExists = true;
            intersection.t = t;
            intersection.intersection = P;
            intersection.normal = m_normal;
            intersection.u = proj1 / width;
            intersection.v = proj2 / height;
        }

        return intersection;
    }
};
#endif
