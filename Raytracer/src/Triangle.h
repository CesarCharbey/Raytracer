#ifndef TRIANGLE_H
#define TRIANGLE_H
#include "Vec3.h"
#include "Ray.h"
#include "Plane.h"
#include <cfloat>
#include <cmath>

struct RayTriangleIntersection
{
    bool intersectionExists;
    float t;
    float w0, w1, w2;
    unsigned int tIndex;
    Vec3 intersection;
    Vec3 normal;
};

class Triangle
{
private:
    Vec3 m_c[3], m_normal;
    float area;

public:
    Triangle() {}
    Triangle(Vec3 const &c0, Vec3 const &c1, Vec3 const &c2)
    {
        m_c[0] = c0;
        m_c[1] = c1;
        m_c[2] = c2;
        updateAreaAndNormal();
    }
    void updateAreaAndNormal()
    {
        Vec3 nNotNormalized = Vec3::cross(m_c[1] - m_c[0], m_c[2] - m_c[0]);
        float norm = nNotNormalized.length();
        m_normal = nNotNormalized / norm;
        area = norm / 2.f;
    }
    void setC0(Vec3 const &c0) { m_c[0] = c0; } // remember to update the area and normal afterwards!
    void setC1(Vec3 const &c1) { m_c[1] = c1; } // remember to update the area and normal afterwards!
    void setC2(Vec3 const &c2) { m_c[2] = c2; } // remember to update the area and normal afterwards!
    Vec3 const &normal() const { return m_normal; }
    Vec3 projectOnSupportPlane(Vec3 const &p) const
    {
        Vec3 result;
        // TODO completer
        return result;
    }
    float squareDistanceToSupportPlane(Vec3 const &p) const
    {
        float result;
        // TODO completer
        return result;
    }
    float distanceToSupportPlane(Vec3 const &p) const { return sqrt(squareDistanceToSupportPlane(p)); }
    bool isParallelTo(Line const &L) const
    {
        bool result;
        // TODO completer
        return result;
    }
    Vec3 getIntersectionPointWithSupportPlane(Line const &L) const
    {
        // you should check first that the line is not parallel to the plane!
        Vec3 result;
        // TODO completer
        return result;
    }
    void computeBarycentricCoordinates(Vec3 const &p, float &u0, float &u1, float &u2) const
    {
        // TODO Complete
    }

    RayTriangleIntersection getIntersection(Ray const &ray) const
    {
        RayTriangleIntersection r;
        r.intersectionExists = false;
        r.t = FLT_MAX;
        r.w0 = r.w1 = r.w2 = 0.f;
        r.tIndex = 0;
        r.intersection = Vec3(0.f, 0.f, 0.f);
        r.normal = m_normal;

        const Vec3 ro = ray.origin();
        const Vec3 rd = ray.direction(); // supposé unitaire
        const float eps = 1e-8f;

        // Möller–Trumbore
        Vec3 e1 = m_c[1] - m_c[0];
        Vec3 e2 = m_c[2] - m_c[0];
        Vec3 pvec = Vec3::cross(rd, e2);
        float det = Vec3::dot(e1, pvec);

        if (std::fabs(det) < eps)
            return r; // no-cull: rayon // au plan

        float invDet = 1.0f / det;
        Vec3 tvec = ro - m_c[0];
        float u = Vec3::dot(tvec, pvec) * invDet;
        if (u < -1e-6f || u > 1.0f + 1e-6f)
            return r;

        Vec3 qvec = Vec3::cross(tvec, e1);
        float v = Vec3::dot(rd, qvec) * invDet;
        if (v < -1e-6f || u + v > 1.0f + 1e-6f)
            return r;

        float t = Vec3::dot(e2, qvec) * invDet;
        if (t <= 1e-6f)
            return r; // derrière ou trop proche

        r.intersectionExists = true;
        r.t = t;
        r.w1 = u;
        r.w2 = v;
        r.w0 = 1.f - u - v;
        r.intersection = ro + t * rd;
        r.normal = m_normal; // Mesh::intersect mettra la normale finale (flat ou lissée)
        return r;
    }
};
#endif
