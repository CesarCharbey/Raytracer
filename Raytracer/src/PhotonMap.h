// PhotonMap.h
#ifndef PHOTONMAP_H
#define PHOTONMAP_H

#include <vector>
#include <cmath>
#include "Vec3.h"

#include <GL/glut.h>
#include "PhotonKdTree.h"

class PhotonMap
{
public:
    std::vector<Photon> photons;
    PhotonKdTree kdtree;
    bool isBuilt = false;

    void clear() // Permet de vider la photon map
    {
        photons.clear();
    }

    void store(const Photon &p) // Permet de stocker un photon
    {
        photons.push_back(p);
    }

    void build()
    {
        kdtree.build(photons);
        isBuilt = true;
    }

    // Recherche les photons autour de P dans un rayon
    Vec3 estimateCaustics(const Vec3 &P, float radius)
    {
        if (!isBuilt && !photons.empty())
            build();
        if (photons.empty())
            return Vec3(0, 0, 0);

        Vec3 flux(0, 0, 0);
        // Buffer static pour éviter les allocations mémoire à chaque pixel
        static std::vector<int> results;
        results.clear();
        kdtree.search(P, radius, results);
        if (results.empty())
            return Vec3(0, 0, 0);

        // Cone filter, (algorithme de Jensen), + de poids aux photons proches du centre
        float k = 1.1f;
        for (int idx : results)
        {
            const Photon &ph = photons[idx];
            float dist = (ph.position - P).length();
            // Poids w = 1 - (d / (k * r)
            float weight = 1.0f - (dist / (k * radius));
            if (weight > 0.f)
            {
                flux += ph.power * weight;
            }
        }
        // Normalisation pour le filtre Cone : (1 - 2/3k) * PI * r^2
        float normalization = (1.0f - 2.0f / (3.0f * k));
        float area = M_PI * radius * radius;
        return flux / (normalization * area);
    }

    // dessiner les photons (optionnel a appeller dans draw())
    void debugDrawGL()
    {
        glBegin(GL_POINTS);
        for (const auto &p : photons)
        {
            Vec3 displayColor = p.power * 5000.0f;

            glColor3f(std::min(1.f, displayColor[0]),
                      std::min(1.f, displayColor[1]),
                      std::min(1.f, displayColor[2]));

            glVertex3f(p.position[0], p.position[1], p.position[2]);
        }
        glEnd();
    }
};

#endif