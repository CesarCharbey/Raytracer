#ifndef Sphere_H
#define Sphere_H
#include "Vec3.h"
#include <vector>
#include "Mesh.h"
#include <cmath>

struct RaySphereIntersection
{
    bool intersectionExists;
    float t;
    float theta, phi;
    float u = 0.f, v = 0.f; // uv
    Vec3 intersection;
    Vec3 secondintersection;
    Vec3 normal;
};

static Vec3 SphericalCoordinatesToEuclidean(Vec3 ThetaPhiR)
{
    return ThetaPhiR[2] * Vec3(cos(ThetaPhiR[0]) * cos(ThetaPhiR[1]), sin(ThetaPhiR[0]) * cos(ThetaPhiR[1]), sin(ThetaPhiR[1]));
}
static Vec3 SphericalCoordinatesToEuclidean(float theta, float phi)
{
    return Vec3(cos(theta) * cos(phi), sin(theta) * cos(phi), sin(phi));
}

static Vec3 EuclideanCoordinatesToSpherical(Vec3 xyz)
{
    float R = xyz.length();
    float phi = asin(xyz[2] / R);
    float theta = atan2(xyz[1], xyz[0]);
    return Vec3(theta, phi, R);
}

class Sphere : public Mesh
{
public:
    Vec3 m_center;
    float m_radius;

    Sphere() : Mesh() {}
    Sphere(Vec3 c, float r) : Mesh(), m_center(c), m_radius(r) {}

    void build_arrays()
    {
        unsigned int nTheta = 20, nPhi = 20;
        positions_array.resize(3 * nTheta * nPhi);
        normalsArray.resize(3 * nTheta * nPhi);
        uvs_array.resize(2 * nTheta * nPhi);
        for (unsigned int thetaIt = 0; thetaIt < nTheta; ++thetaIt)
        {
            float u = (float)(thetaIt) / (float)(nTheta - 1);
            float theta = u * 2 * M_PI;
            for (unsigned int phiIt = 0; phiIt < nPhi; ++phiIt)
            {
                unsigned int vertexIndex = thetaIt + phiIt * nTheta;
                float v = (float)(phiIt) / (float)(nPhi - 1);
                float phi = -M_PI / 2.0 + v * M_PI;
                Vec3 xyz = SphericalCoordinatesToEuclidean(theta, phi);
                positions_array[3 * vertexIndex + 0] = m_center[0] + m_radius * xyz[0];
                positions_array[3 * vertexIndex + 1] = m_center[1] + m_radius * xyz[1];
                positions_array[3 * vertexIndex + 2] = m_center[2] + m_radius * xyz[2];
                normalsArray[3 * vertexIndex + 0] = xyz[0];
                normalsArray[3 * vertexIndex + 1] = xyz[1];
                normalsArray[3 * vertexIndex + 2] = xyz[2];
                uvs_array[2 * vertexIndex + 0] = u;
                uvs_array[2 * vertexIndex + 1] = v;
            }
        }
        triangles_array.clear();
        for (unsigned int thetaIt = 0; thetaIt < nTheta - 1; ++thetaIt)
        {
            for (unsigned int phiIt = 0; phiIt < nPhi - 1; ++phiIt)
            {
                unsigned int vertexuv = thetaIt + phiIt * nTheta;
                unsigned int vertexUv = thetaIt + 1 + phiIt * nTheta;
                unsigned int vertexuV = thetaIt + (phiIt + 1) * nTheta;
                unsigned int vertexUV = thetaIt + 1 + (phiIt + 1) * nTheta;
                triangles_array.push_back(vertexuv);
                triangles_array.push_back(vertexUv);
                triangles_array.push_back(vertexUV);
                triangles_array.push_back(vertexuv);
                triangles_array.push_back(vertexUV);
                triangles_array.push_back(vertexuV);
            }
        }
    }

    RaySphereIntersection intersect(const Ray &ray) const
    {
        RaySphereIntersection intersection;
        // TODO calcul l'intersection rayon sphere
        intersection.intersectionExists = false;

        Vec3 origineRayon = ray.origin();
        Vec3 directionRayon = ray.direction();
        directionRayon.normalize();
        Vec3 centreSphere = m_center;
        float rayonSphere = m_radius;

        Vec3 origineVersCentre = origineRayon - centreSphere;
        float a = Vec3::dot(directionRayon, directionRayon);
        float b = 2.0f * Vec3::dot(origineVersCentre, directionRayon);
        float c = Vec3::dot(origineVersCentre, origineVersCentre) - rayonSphere * rayonSphere;

        float discriminant = b * b - 4 * a * c; // b² - 4ac
        if (discriminant < 0)
        {
            return intersection; // Pas d'intersection
        }

        float racineDiscriminant = sqrt(discriminant); // on calcule la racine du discriminant
        float t1 = (-b - racineDiscriminant) / (2 * a);
        float t2 = (-b + racineDiscriminant) / (2 * a);

        float tIntersection = t1; // on choisit la plus proche intersection
        if (tIntersection < 0)
            tIntersection = t2;
        if (tIntersection < 0)
            return intersection; // cas où les deux intersections sont derrière l'origine du rayon

        intersection.intersectionExists = true;
        intersection.t = tIntersection;
        intersection.intersection = origineRayon + tIntersection * directionRayon; // pt d'intersection sur la sphère
        intersection.normal = (intersection.intersection - centreSphere);
        intersection.normal.normalize();

        Vec3 n = intersection.normal;                                 // normalisée
        intersection.u = (atan2f(n[2], n[0]) + M_PI) / (2.f * M_PI);  // u [0,1) autour de l’axe Y (longitude)
        intersection.v = acosf(std::clamp(n[1], -1.0f, 1.0f)) / M_PI; // v [0,1] du pole nord (v=0) au pole sud (v=1)

        // on calcule theta et phi qui sont les coordonnées équivalent au point d'intersection sur la sphère
        Vec3 coordSpheriques = EuclideanCoordinatesToSpherical(intersection.normal);
        intersection.theta = coordSpheriques[0];
        intersection.phi = coordSpheriques[1];

        intersection.secondintersection = origineRayon + t2 * directionRayon; // calcul du 2ime point d'intersection

        return intersection;
    }
};
#endif
