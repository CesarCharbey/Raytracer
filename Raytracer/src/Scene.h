#ifndef SCENE_H
#define SCENE_H

#include <vector>
#include <string>
#include "Mesh.h"
#include "Sphere.h"
#include "Square.h"
#include <random>
#include <limits>
#include <cstdint>
#include <cstring>
#include <algorithm>

#include "KdTree.h"    // pour l'accélération globale
#include "PhotonMap.h" // pour la photon map

#include <GL/glut.h>

// énumération des types de lumière
enum LightType
{
    LightType_Spherical,
    LightType_Quad
};

// structure représentant une source lumineuse
struct Light
{
    Vec3 material;
    bool isInCamSpace;
    LightType type;

    Vec3 pos;
    float radius;

    Mesh quad;

    float powerCorrection;

    Light() : powerCorrection(1.0) {}

    // pour les ombres douces
    Vec3 u, v;
    int samples = 32;
};

// structure stockant les infos d'intersection
struct RaySceneIntersection
{
    bool intersectionExists;
    unsigned int typeOfIntersectedObject;
    unsigned int objectIndex;
    float t;
    RayTriangleIntersection rayMeshIntersection;
    RaySphereIntersection raySphereIntersection;
    RaySquareIntersection raySquareIntersection;
    RaySceneIntersection() : intersectionExists(false), t(FLT_MAX) {}
};

// approximation de schlick pour fresnel
inline float fresnel_schlick(float cosTheta, float etai, float etat)
{
    float r0 = (etai - etat) / (etai + etat);
    r0 *= r0;
    return r0 + (1.f - r0) * std::pow(1.f - cosTheta, 5.f);
}

// contraint une valeur entre 0 et 1
inline float clamp01(float x) { return x < 0.f ? 0.f : (x > 1.f ? 1.f : x); }

// calcule la direction réfléchie
inline Vec3 reflectDir(const Vec3 &I, const Vec3 &N) { return I - 2.f * Vec3::dot(I, N) * N; }

// calcule la direction réfractée (snell-descartes)
inline bool refractDir(const Vec3 &I, const Vec3 &N, float eta, Vec3 &T)
{
    float cosi = -Vec3::dot(I, N);
    float k = 1.f - eta * eta * (1.f - cosi * cosi);
    if (k < 0.f)
        return false;
    T = eta * I + (eta * cosi - std::sqrt(k)) * N;
    return true;
}

class Scene
{
    std::vector<Mesh> meshes;
    std::vector<Sphere> spheres;
    std::vector<Square> squares;

private:
    KdTree kd;
    bool dbgShowKdGlobal = false;
    bool dbgShowKdMeshes = false;
    int dbgKdMaxDepth = -1; // -1 => tout
    bool dbgKdLeavesOnly = false;

public:
    Scene()
    {
    }

    // on met public pour pouvoir y accéder depuis main.cpp et imGui
    std::vector<Light> lights;
    std::vector<Vec3> initialLightPos;
    bool showLights = true;

    // sauvegarde position initiale lumières
    void saveInitialLightState()
    {
        initialLightPos.clear();
        for (const auto &l : lights)
        {
            initialLightPos.push_back(l.pos);
        }
    }

    // réinitialise position lumières
    void resetLightPositions()
    {
        for (size_t i = 0; i < lights.size() && i < initialLightPos.size(); ++i)
        {
            lights[i].pos = initialLightPos[i];
        }
    }

    PhotonMap photonMap;

    bool showPhotons = false; // vairables pour le debug : appuyer sur 'p' pour afficher la photon map
    bool showKdTree = false;  // 'k' pour affichier le kd-tree
    bool isUnderwaterScene = false;

    Vec3 randomDirection() // direction aléatoire unifrome sur la sphère
    {
        float x, y, z;
        do
        {
            x = 2.0f * (float)rand() / RAND_MAX - 1.0f;
            y = 2.0f * (float)rand() / RAND_MAX - 1.0f;
            z = 2.0f * (float)rand() / RAND_MAX - 1.0f;
        } while (x * x + y * y + z * z > 1.0f); // Rejet si hors de la sphère
        Vec3 res(x, y, z);
        res.normalize();
        return res;
    }

    // configuration debug kd-tree
    void setKdDebug(bool showGlobal, bool showMeshes, int maxDepth = -1, bool leavesOnly = false)
    {
        dbgShowKdGlobal = showGlobal;
        dbgShowKdMeshes = showMeshes;
        dbgKdMaxDepth = maxDepth;
        dbgKdLeavesOnly = leavesOnly;
    }

    void draw()
    {
        // --- rendu normal des objets ---
        for (unsigned int It = 0; It < meshes.size(); ++It)
            meshes[It].draw();
        for (unsigned int It = 0; It < spheres.size(); ++It)
            spheres[It].draw();
        for (unsigned int It = 0; It < squares.size(); ++It)
            squares[It].draw();

        // rendu des lumières
        if (showLights)
        {
            for (const Light &light : lights)
            {
                glPushMatrix();
                // On se déplace à la position de la lumière
                glTranslatef(light.pos[0], light.pos[1], light.pos[2]);
                // On désactive la lumière pour que l'objet soit "lumineux" (couleur pure)
                glDisable(GL_LIGHTING);
                // Couleur Jaune (R=1, G=1, B=0)
                glColor3f(1.0f, 1.0f, 0.0f);
                // On dessine une petite sphère représentative
                // (Rayon 0.1, 10 méridiens, 10 parallèles)
                glutWireSphere(0.1, 10, 10);
                // On remet la lumière pour les objets suivants
                glEnable(GL_LIGHTING);
                glPopMatrix();
            }
        }
        // debug KD-tree
        if (showKdTree)
        {
            glPushAttrib(GL_ENABLE_BIT | GL_LINE_BIT | GL_CURRENT_BIT);
            glDisable(GL_LIGHTING);
            glDisable(GL_DEPTH_TEST);
            glLineWidth(2.0f);

            // 8,false = 8 niveaux
            // -1, false = tous les niveaux
            // -1, true = toutes les feuilles
            kd.debugDraw(-1, false); // top-level : tous les niveaux
            for (const auto &m : meshes)
                m.debugDrawKd(-1, false); // kd interne des meshes : 2 niveaux

            glPopAttrib();
        }

        // Debug photon map
        if (showPhotons)
        {
            glDisable(GL_LIGHTING);
            glPointSize(1.0f); // Points un peu plus fins si on en affiche beaucoup
            glBegin(GL_POINTS);
            // On prend le minimum entre le nombre réel et 100 000
            size_t maxDisplay = 100000;
            size_t count = std::min(maxDisplay, photonMap.photons.size());
            for (size_t i = 0; i < count; ++i)
            {
                const auto &p = photonMap.photons[i];
                glColor3f(1.0f, 1.0f, 1.0f);
                glVertex3f(p.position[0], p.position[1], p.position[2]);
            }
            glEnd();

            glEnable(GL_LIGHTING);
        }
    }

    // calcul de l'intersection la plus proche
    RaySceneIntersection computeIntersection(Ray const &ray)
    {
        RaySceneIntersection best;
        best.intersectionExists = false;
        best.t = std::numeric_limits<float>::max();

        // Pré-test carrés -> cutoff
        for (size_t i = 0; i < squares.size(); ++i)
        {
            RaySquareIntersection sq = squares[i].intersect(ray);
            if (sq.intersectionExists && sq.t > 1e-4f && sq.t < best.t)
            {
                best.intersectionExists = true;
                best.t = sq.t;
                best.typeOfIntersectedObject = 0;
                best.objectIndex = (unsigned)i;
                best.raySquareIntersection = sq;
            }
        }

        // KD avec cutoff best.t (élagage agressif du mesh)
        if (!kd.empty())
        {
            KdHit kh = kd.intersect(ray, best.t);
            if (kh.hit && kh.t < best.t)
            {
                best.intersectionExists = true;
                best.t = kh.t;
                if (kh.type == 0)
                {
                    best.typeOfIntersectedObject = 0;
                    best.objectIndex = kh.index;
                    best.raySquareIntersection = kh.squareHit;
                }
                else if (kh.type == 1)
                {
                    best.typeOfIntersectedObject = 1;
                    best.objectIndex = kh.index;
                    best.raySphereIntersection = kh.sphereHit;
                }
                else
                {
                    best.typeOfIntersectedObject = 2;
                    best.objectIndex = kh.index;
                    best.rayMeshIntersection = kh.triHit;
                }
            }
        }
        return best;
    }

    // pour les ombres douces on génère un float aléatoire entre 0 et 1
    inline float rnd01()
    {
        return static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    }

    // point aléatoire sur le carré lumineux (uniforme)
    inline Vec3 sampleOnAreaLight(const Light &L)
    {
        // xi, yi dans [-1, 1]
        float xi = 2.f * rnd01() - 1.f;
        float yi = 2.f * rnd01() - 1.f;
        return L.pos + xi * L.u + yi * L.v;
    }

    // coeur du raytracer récursif
    Vec3 rayTraceRecursive(const Ray &ray, int profondeur)
    {
        // condition d'arrêt recursion
        if (profondeur <= 0)
        {
            return Vec3(0.f, 0.f, 0.f);
        }

        // calcul intersection rayon/scène
        RaySceneIntersection intersection = computeIntersection(ray);
        if (!intersection.intersectionExists)
        {
            return Vec3(0.0f, 0.0f, 0.0f); // si pas d'objet alors on renvoie du noir
        }

        Material mat;
        Vec3 N; // normale au point d'inter
        Vec3 P; // pt d'inter

        // récupération infos selon type objet
        switch (intersection.typeOfIntersectedObject)
        {
        case 0: // carré
            mat = squares[intersection.objectIndex].material;
            N = intersection.raySquareIntersection.normal * -1;
            P = intersection.raySquareIntersection.intersection;
            break;
        case 1: // sphere
            mat = spheres[intersection.objectIndex].material;
            N = intersection.raySphereIntersection.normal;
            P = intersection.raySphereIntersection.intersection;
            break;
        case 2: // mesh
            mat = meshes[intersection.objectIndex].material;
            N = intersection.rayMeshIntersection.normal;
            P = intersection.rayMeshIntersection.intersection;
            break;
        default:
            return Vec3(0.f, 0.f, 0.f);
        }
        Vec3 Kd = mat.diffuse_material;  // coeff de reflexion materiaux diff
        Vec3 Ks = mat.specular_material; // coeff de reflexion speculaire
        float shininess = mat.shininess;

        Vec3 ambientLight; // I_sa
        // ajustement lumière ambiante si sous l'eau
        if (isUnderwaterScene)
        {
            if (P[1] > 0.0f)
            {
                ambientLight = Vec3(0.3f, 0.3f, 0.3f); // Plus clair dans l'air
            }
            else
            {
                ambientLight = Vec3(0.1f, 0.1f, 0.1f); // Plus sombre sous l'eau
            }
        }
        else
        {
            ambientLight = Vec3(0.1f, 0.1f, 0.1f);
        }

        // Gestion pour l'eau
        N.normalize();
        Vec3 Ia = ambientLight * Kd;
        Vec3 Id(0.f, 0.f, 0.f);
        Vec3 Is(0.f, 0.f, 0.f);

        const float epsilon = 1e-3f;
        Vec3 Position_offset = P + N * epsilon;
        Vec3 V = -ray.direction(); // vecteur vue
        V.normalize();

        // boucle sur les sources lumineuses
        for (const Light &light : lights)
        {
            int S = std::max(1, light.samples); // Nombre d'échantillons
            Vec3 Isd = light.material;          // Couleur de la lumière
            Vec3 Id_local(0.f, 0.f, 0.f);
            Vec3 Is_local(0.f, 0.f, 0.f);

            // boucle echantillonnage ombres douces
            for (int s = 0; s < S; s++)
            {
                // Echantillonnage sur la surface de la lumière
                Vec3 Pl = sampleOnAreaLight(light);

                // Vecteur et distance
                Vec3 Ldir = Pl - P;
                float Ldist = Ldir.length();
                Ldir /= Ldist; // Normalisation
                Ray shadowRay(Position_offset, Ldir);
                bool occluded = false;

                // Utilisation du KdTree pour l'ombre
                if (!kd.empty())
                {
                    occluded = kd.anyHitBefore(shadowRay, Ldist - 1e-4f);
                }
                else
                {
                    RaySceneIntersection shadowIntersection = computeIntersection(shadowRay);
                    if (shadowIntersection.intersectionExists && shadowIntersection.t < Ldist - 1e-4f)
                    {
                        occluded = true;
                    }
                }

                if (occluded)
                    continue; // Ce rayon est bloqué, on passe au suivant

                // Calcul Diffus
                float dotNL = std::max(0.f, Vec3::dot(N, Ldir));
                Id_local += Isd * Kd * dotNL;

                // Calcul Spéculaire (Modèle Phong)
                /*Vec3 R = 2.f * Vec3::dot(N, Ldir) * N - Ldir; // Reflet de la lumière
                R.normalize();
                float dotRV = std::max(0.f, Vec3::dot(R, V));
                Is_local += Isd * Ks * std::pow(dotRV, shininess);*/

                // Calcul Spéculaire (Modèle Blinn-Phong)
                Vec3 H = Ldir + V; // Vecteur median, somme lumière + vue
                H.normalize();
                float dotNH = std::max(0.f, Vec3::dot(N, H));
                Is_local += Isd * Ks * std::pow(dotNH, shininess * 4.0f); // shininess *4 pour Blinn-Phong
            }

            // Moyenne des échantillons
            float invS = 1.f / static_cast<float>(S);
            Id += Id_local * invS;
            Is += Is_local * invS;
        }

        Vec3 color = Ia + Id + Is;

        Vec3 I = ray.direction();
        I.normalize();
        Vec3 Ng = N;
        if (Vec3::dot(I, N) > 0.f)
        {
            Ng = -N;
        }
        // Normale orientée (pour entrer/sortir du verre)
        Vec3 Nrefl = (Vec3::dot(I, N) < 0.f) ? N : -N;
        Vec3 Rdir = reflectDir(I, Nrefl);

        Vec3 Cr(0.f, 0.f, 0.f); // Couleur Refléchie

        bool isTransparent = (mat.type == Material_Glass || mat.type == Material_Water); // gestion similaire pour verre et eau
        // calcul récursif réflexion
        if (profondeur > 0 && (mat.type == Material_Mirror || isTransparent))
        {
            Ray rRay(P + epsilon * Rdir, Rdir);
            Cr = rayTraceRecursive(rRay, profondeur - 1);
        }

        if (mat.type == Material_Mirror)
        {
            // miroir presque parfait
            float mirrorStrength = 0.9f;
            color = (1.f - mirrorStrength) * color + mirrorStrength * Cr;
        }
        else if (isTransparent)
        {
            float etai = 1.f, etat = (mat.index_medium > 0.f ? mat.index_medium : 1.33f);
            float cosi = Vec3::dot(I, N); // On utilise la normale originale pour le test de sens

            Vec3 N_refract = N;
            if (cosi > 0.f) // On sort du milieu
            {
                std::swap(etai, etat);
                N_refract = -N;
                cosi = -cosi; // cos theta doit être positif pour les formules
            }

            float eta = etai / etat;
            Vec3 Tdir;
            Vec3 Ct(0.f, 0.f, 0.f);
            // TENTATIVE DE REFRACTION
            bool hasRefract = refractDir(I, N_refract, eta, Tdir);
            // Fresnel (Schlick)
            float Rf = fresnel_schlick(std::fabs(cosi), etai, etat);

            // lancer rayon réfracté
            if (hasRefract && profondeur > 0)
            {
                Tdir.normalize();
                Ray tRay(P - epsilon * N_refract, Tdir);
                Ct = rayTraceRecursive(tRay, profondeur - 1);
                Vec3 tint = mat.diffuse_material;
                Ct = Vec3(Ct[0] * tint[0], Ct[1] * tint[1], Ct[2] * tint[2]);
            }
            else
            {
                // Réflexion totale interne : Tout est réfléchi (Rf = 1.0)
                Rf = 1.0f;
            }

            // mix réflexion/réfraction selon fresnel
            color = Rf * Cr + (1.f - Rf) * Ct;

            // ajoute écume sommets vagues
            if (mat.type == Material_Water)
            {
                float waveSteepness = 1.0f - std::abs(Vec3::dot(N, Vec3(0, 1, 0)));
                if (waveSteepness > 0.05f)
                {
                    color += Vec3(0.2f, 0.2f, 0.2f) * waveSteepness;
                }
            }
        }
        else
        {
            // Matériau diffus standard : on ajoute les caustiques (Photon Map)
            float searchRadius = isUnderwaterScene ? 0.02f : 0.08f;
            Vec3 caustics = photonMap.estimateCaustics(P, searchRadius);
            color += caustics * mat.diffuse_material;
        }

        // EFFET ESTHETIQUE : BROUILLARD DANS L'EAU, utile pour la scène avec de l'eau, mais peut être désactivé si besoin
        if (isUnderwaterScene && !isTransparent)
        {
            float waterHeight = 0.0f; // La hauteur Y où tu as placé ton Mesh d'eau

            // Cas 1 : SOUS L'EAU (P.y < waterHeight)
            if (P[1] < waterHeight)
            {
                // -- Ton joli dégradé bleu marin --
                Vec3 surfaceFog = Vec3(0.2f, 0.4f, 0.6f);
                Vec3 deepFog = Vec3(0.01f, 0.04f, 0.10f);

                // Facteur de profondeur (0 au fond, 1 à la surface)
                float heightFactor = (P[1] + 2.0f) / 2.0f;
                heightFactor = clamp01(heightFactor);

                Vec3 finalFogColor = deepFog * (1.0f - heightFactor) + surfaceFog * heightFactor;

                // Densité forte sous l'eau
                float fogDensity = 0.3f;
                float fogfactor = 1.0f - std::exp(-fogDensity * intersection.t);

                color = (1.0f - fogfactor) * color + fogfactor * finalFogColor;
            }
            // Cas 2 : DANS L'AIR (P.y >= waterHeight)
            else
            {
                // Brouillard très léger ou nul
                // Ici, on met juste un très léger assombrissement au fond de la boite
                // pour ne pas que ce soit noir total, mais on garde la clarté.

                Vec3 airColor = Vec3(0.0f, 0.0f, 0.0f); // Noir (assombrissement naturel)
                float airDensity = 0.02f;               // Très faible densité !

                float fogfactor = 1.0f - std::exp(-airDensity * intersection.t);
                color = (1.0f - fogfactor) * color + fogfactor * airColor;
            }
        }

        // ACES Tone Mapping, pour un meilleur rendu des hautes lumières / contrastes
        color = aces_approx(color);
        // clamp les couleurs entre 0 et 1 juste pour la sécurité mais globalement inutile avec le tone mapping
        color[0] = clamp01(color[0]);
        color[1] = clamp01(color[1]);
        color[2] = clamp01(color[2]);
        return color;
    }

    // Fonction utilitaire pour le Tone Mapping ACES
    Vec3 aces_approx(Vec3 v)
    {
        v *= 0.6f;
        float a = 2.51f;
        float b = 0.03f;
        float c = 2.43f;
        float d = 0.59f;
        float e = 0.14f;
        float x = (v[0] * (a * v[0] + b)) / (v[0] * (c * v[0] + d) + e);
        float y = (v[1] * (a * v[1] + b)) / (v[1] * (c * v[1] + d) + e);
        float z = (v[2] * (a * v[2] + b)) / (v[2] * (c * v[2] + d) + e);

        return Vec3(x, y, z);
    }

    // point d'entrée raytracing
    Vec3 rayTrace(Ray const &rayStart)
    {
        // TODO appeler la fonction recursive
        /*RaySceneIntersection intersection = computeIntersection(rayStart);
        Vec3 color(0, 0, 0);
        if (intersection.intersectionExists)
        {
            switch (intersection.typeOfIntersectedObject)
            {
            case 0: // Carré
                if (intersection.objectIndex < squares.size())
                    color = squares[intersection.objectIndex].material.diffuse_material;
                break;
            case 1: // Sphere
                if (intersection.objectIndex < spheres.size())
                    color = spheres[intersection.objectIndex].material.diffuse_material;
                break;
            case 2: // maillage
                if (intersection.objectIndex < meshes.size())
                    color = meshes[intersection.objectIndex].material.diffuse_material;
                break;
            default:
                break;
            }
        }
        return color;*/
        return rayTraceRecursive(rayStart, 5);
    }

    // lancer de photons pour caustiques
    void tracePhoton(const Ray &ray, const Vec3 &power, int depth, bool isSpecular)
    {
        if (depth > 5)
            return; // Limite de rebonds
        if (power.length() < 1e-7f)
            return; // Élagage si puissance trop faible

        RaySceneIntersection intersection = computeIntersection(ray);
        if (!intersection.intersectionExists)
            return;
        // Récupération du matériau
        Material mat;
        if (intersection.typeOfIntersectedObject == 0)
            mat = squares[intersection.objectIndex].material;
        else if (intersection.typeOfIntersectedObject == 1)
            mat = spheres[intersection.objectIndex].material;
        else
            mat = meshes[intersection.objectIndex].material;

        Vec3 P = intersection.typeOfIntersectedObject == 0 ? intersection.raySquareIntersection.intersection : (intersection.typeOfIntersectedObject == 1 ? intersection.raySphereIntersection.intersection : intersection.rayMeshIntersection.intersection);
        Vec3 N = intersection.typeOfIntersectedObject == 0 ? intersection.raySquareIntersection.normal : (intersection.typeOfIntersectedObject == 1 ? intersection.raySphereIntersection.normal : intersection.rayMeshIntersection.normal);
        N.normalize();

        // Logique du photon selon le matériau
        if (mat.type == Material_Glass || mat.type == Material_Water)
        {
            // Réfraction / Réflexion (Spéculaire) -> Le photon rebondit, on stocke RIEN
            float etai = 1.f, etat = (mat.index_medium > 0.f ? mat.index_medium : 1.5f);
            Vec3 I = ray.direction();
            float cosi = Vec3::dot(I, N);
            Vec3 Ng = N;
            if (cosi > 0.f)
            {
                std::swap(etai, etat);
                Ng = -N;
            } // Sortie du verre

            float eta = etai / etat;
            float k = 1.f - eta * eta * (1.f - cosi * cosi);

            // Fresnel
            float R_prob = fresnel_schlick(std::fabs(cosi), etai, etat);
            // nombre aléatoire entre 0 et 1
            float rng = rnd01();
            if (k < 0.f || rng < R_prob) // Réflexion totale ou pas de chance (Fresnel)
            {
                // Réflexion totale interne
                Vec3 R = reflectDir(I, Ng);
                tracePhoton(Ray(P + 1e-4f * R, R), power, depth + 1, true);
            }
            else
            {
                // Réfraction
                Vec3 T = eta * I + (eta * cosi - std::sqrt(k)) * Ng;
                T.normalize();
                tracePhoton(Ray(P + 1e-4f * T, T), power, depth + 1, true);
            }
        }
        else if (mat.type == Material_Mirror)
        {
            // Réflexion pure
            Vec3 R = reflectDir(ray.direction(), N);
            tracePhoton(Ray(P + 1e-4f * R, R), power, depth + 1, false);
        }
        else
        {
            // Surface Diffuse (Mur, Sol, etc.)
            // Condition : Le photon doit venir d'un rebond spéculaire (depth > 0)
            // Sinon c'est juste de l'éclairage direct
            if (depth > 0 && isSpecular)
            {
                Photon p;
                p.position = P;
                p.power = power; // Couleur du photon
                p.incident = ray.direction();
                photonMap.store(p);
            }
        }
    }

    // Dans Scene.h
    void emitPhotons(int nbPhotons)
    {
        if (lights.empty())
            return;
        photonMap.clear();
        std::cout << "Emission de " << nbPhotons << " photons..." << std::endl;
        const Light &l = lights[0];
        float intensityFactor = 50.0f;
        Vec3 photonPower = l.material * l.powerCorrection * intensityFactor * (1.0f / nbPhotons);

        // boucle émission photons depuis lumière
        for (int i = 0; i < nbPhotons; ++i)
        {
            Vec3 origin = sampleOnAreaLight(l); // Point aléatoire sur la lumière
            Vec3 dir = randomDirection();       // Direction aléatoire
            tracePhoton(Ray(origin, dir), photonPower, 0, false);
        }
        std::cout << "Photons stockes : " << photonMap.photons.size() << std::endl;
        photonMap.build();
        std::cout << "PhotonKdTree construit." << std::endl;
    }

    // scène test : sphère unique
    void setup_single_sphere()
    {
        isUnderwaterScene = false;
        meshes.clear();
        spheres.clear();
        squares.clear();
        lights.clear();

        {
            lights.resize(lights.size() + 1);
            Light &light = lights[lights.size() - 1];
            light.pos = Vec3(-5, 5, 5);
            light.radius = 2.5f;
            light.powerCorrection = 2.f;
            light.type = LightType_Spherical;
            light.material = Vec3(1, 1, 1);
            light.isInCamSpace = false;
        }
        {
            spheres.resize(spheres.size() + 1);
            Sphere &s = spheres[spheres.size() - 1];
            s.m_center = Vec3(0., 0., 0.);
            s.m_radius = 1.f;
            s.build_arrays();
            s.material.type = Material_Mirror;
            s.material.diffuse_material = Vec3(1., 1., 1);
            s.material.specular_material = Vec3(0.2, 0.2, 0.2);
            s.material.shininess = 20;
        }
        kd.build(&squares, &spheres, &meshes, 1, 32);
        saveInitialLightState();
    }

    // scène test : carré unique
    void setup_single_square()
    {
        isUnderwaterScene = false;
        meshes.clear();
        spheres.clear();
        squares.clear();
        lights.clear();

        {
            lights.resize(lights.size() + 1);
            Light &light = lights[lights.size() - 1];
            light.pos = Vec3(-5, 5, 5);
            light.radius = 2.5f;
            light.powerCorrection = 2.f;
            light.type = LightType_Spherical;
            light.material = Vec3(1, 1, 1);
            light.isInCamSpace = false;
        }

        {
            squares.resize(squares.size() + 1);
            Square &s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.build_arrays();
            s.material.diffuse_material = Vec3(0.8, 0.8, 0.8);
            s.material.specular_material = Vec3(0.8, 0.8, 0.8);
            s.material.shininess = 20;
        }
        kd.build(&squares, &spheres, &meshes, 1, 32);
        saveInitialLightState();
    }

    // scène cornell box
    void setup_cornell_box()
    {
        isUnderwaterScene = false;
        meshes.clear();
        spheres.clear();
        squares.clear();
        lights.clear();

        {
            lights.resize(lights.size() + 1);
            Light &light = lights[lights.size() - 1];
            light.pos = Vec3(0.0, 1.5, 0.0);
            light.radius = 2.5f;
            light.powerCorrection = 2.f;
            light.type = LightType_Spherical;
            light.material = Vec3(1, 1, 1);
            light.isInCamSpace = false;
            light.u = Vec3(0.3f, 0.f, 0.f);
            light.v = Vec3(0.f, -0.1f, 0.3f);
            light.samples = 32;
        }

        { // Back Wall
            squares.resize(squares.size() + 1);
            Square &s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.scale(Vec3(2., 2., 1.));
            s.translate(Vec3(0., 0., -2.));
            s.build_arrays();
            s.material.diffuse_material = Vec3(1., 1., 1.);
            s.material.specular_material = Vec3(1., 1., 1.);
            s.material.shininess = 16;
        }

        { // Left Wall

            squares.resize(squares.size() + 1);
            Square &s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.scale(Vec3(2., 2., 1.));
            s.translate(Vec3(0., 0., -2.));
            s.rotate_y(90);
            s.build_arrays();
            s.material.diffuse_material = Vec3(1., 0., 0.);
            s.material.specular_material = Vec3(1., 0., 0.);
            s.material.shininess = 16;
        }

        { // Right Wall
            squares.resize(squares.size() + 1);
            Square &s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.translate(Vec3(0., 0., -2.));
            s.scale(Vec3(2., 2., 1.));
            s.rotate_y(-90);
            s.build_arrays();
            s.material.diffuse_material = Vec3(0.0, 1.0, 0.0);
            s.material.specular_material = Vec3(0.0, 1.0, 0.0);
            s.material.shininess = 16;
        }

        { // Floor
            squares.resize(squares.size() + 1);
            Square &s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.translate(Vec3(0., 0., -2.));
            s.scale(Vec3(2., 2., 1.));
            s.rotate_x(-90);
            s.build_arrays();
            s.material.diffuse_material = Vec3(0.4f, 0.4f, 0.4f);
            s.material.specular_material = Vec3(1.0f, 1.0f, 1.0f);
            s.material.shininess = 16;
        }

        { // Ceiling
            squares.resize(squares.size() + 1);
            Square &s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.translate(Vec3(0., 0., -2.));
            s.scale(Vec3(2., 2., 1.));
            s.rotate_x(90);
            s.build_arrays();
            s.material.diffuse_material = Vec3(1.0, 1.0, 1.0);
            s.material.specular_material = Vec3(1.0, 1.0, 1.0);
            s.material.shininess = 16;
        }

        { // Front Wall
            squares.resize(squares.size() + 1);
            Square &s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.translate(Vec3(0., 0., -2.));
            s.scale(Vec3(2., 2., 1.));
            s.rotate_y(180);
            s.build_arrays();
            s.material.diffuse_material = Vec3(1.0, 1.0, 1.0);
            s.material.specular_material = Vec3(1.0, 1.0, 1.0);
            s.material.shininess = 16;
        }

        { // GLASS Sphere
            spheres.resize(spheres.size() + 1);
            Sphere &s = spheres[spheres.size() - 1];
            s.m_center = Vec3(1.0, -1.25, 0.5);
            s.m_radius = 0.75f;
            s.build_arrays();
            s.material.type = Material_Glass;
            s.material.diffuse_material = Vec3(1., 0.9, 0.9);
            s.material.specular_material = Vec3(1., 0., 0.);
            s.material.shininess = 16;
            s.material.transparency = 1.0;
            s.material.index_medium = 1.4;
        }

        { // MIRRORED Sphere
            spheres.resize(spheres.size() + 1);
            Sphere &s = spheres[spheres.size() - 1];
            s.m_center = Vec3(-1.0, -1.25, -0.5);
            s.m_radius = 0.75f;
            s.build_arrays();
            s.material.type = Material_Mirror;
            s.material.diffuse_material = Vec3(1., 1., 1.);
            s.material.specular_material = Vec3(1., 1., 1.);
            s.material.shininess = 16;
            s.material.transparency = 0.;
            s.material.index_medium = 0.;
        }

        { // mesh
            meshes.resize(meshes.size() + 1);
            Mesh &mesh = meshes.back();

            mesh.loadOFF("data/xyzrgb_dragon_100k.off");
            mesh.centerAndScaleToUnit();
            mesh.translate(Vec3(0.0f, 0.2f, -1.0f));
            mesh.build_arrays();
            mesh.material.type = Material_Diffuse_Blinn_Phong;
            mesh.material.diffuse_material = Vec3(0.8f, 0.8f, 1.0f);
            mesh.material.specular_material = Vec3(0.4f, 0.4f, 0.4f);
            mesh.material.shininess = 32.f;
        }

        /*{
            lights.resize(lights.size() + 1);
            Light &light = lights[lights.size() - 1];
            light.pos = Vec3(0.8, 0, -0.4);
            light.radius = 2.5f;
            light.powerCorrection = 2.0f;
            light.type = LightType_Spherical;
            light.material = Vec3(1, 1, 1);
            light.isInCamSpace = false;
            light.u = Vec3(0.1f, 0.f, 0.f);
            light.v = Vec3(0.f, 0.f, 0.1f);
            light.samples = 32;
        }
        { // BIG DRAGON test
            meshes.resize(meshes.size() + 1);
            Mesh &mesh = meshes.back();

            mesh.loadOFF("data/xyzrgb_dragon_100k.off");
            mesh.centerAndScaleToUnit();
            mesh.translate(Vec3(0.0f, -0.7f, 0.0f));
            mesh.scale(Vec3(2.f, 2.f, 2.f));
            mesh.rotate_y(210);
            mesh.build_arrays();
            mesh.material.type = Material_Glass;
            mesh.material.diffuse_material = Vec3(0.95f, 0.95f, 0.95f);
            mesh.material.specular_material = Vec3(1.0f, 1.0f, 1.0f);
            mesh.material.shininess = 60.f;
            mesh.material.index_medium = 1.7f;
            mesh.material.transparency = 1.0f;
        }*/

        kdDebugReset(); // remet tous les compteurs à 0
        kd.build(&squares, &spheres, &meshes, 1, 32);
        saveInitialLightState();
    }

    // Calcul de la hauteur
    float getWaterHeight(float x, float z)
    {
        float y = 0.0f;

        // Vague 1 : Basse fréquence (Houle principale)
        float freq1 = 3.0f;
        float amp1 = 0.10f;
        y += std::sin(x * freq1 + z) * amp1;

        // Vague 2 : Moyenne fréquence (Pour créer le croisement et les caustiques)
        float freq2 = 10.0f;
        float amp2 = 0.03f;
        y += std::cos(x * freq2 - z) * amp2;

        return y;
    }

    // Calcul de la normale mathématique (Dérivée pour 2 vagues)
    Vec3 getWaterNormal(float x, float z)
    {
        float freq1 = 3.0f;
        float amp1 = 0.10f;
        float freq2 = 10.0f;
        float amp2 = 0.03f;

        // Dérivée partielle selon X (pente dx)
        // d/dx(sin(ax)) = a*cos(ax)
        float dfdx = amp1 * freq1 * std::cos(x * freq1 + z) - amp2 * freq2 * std::sin(x * freq2 - z);

        // Dérivée partielle selon Z (pente dz)
        // d/dz(sin(z)) = cos(z)
        // d/dz(cos(-z)) = -sin(-z)*(-1) = sin(-z) -> attention aux signes combinés
        float dfdz = amp1 * 1.0f * std::cos(x * freq1 + z) + amp2 * 1.0f * std::sin(x * freq2 - z);

        // Construction du vecteur normal (-dx, 1, -dz)
        Vec3 N(-dfdx, 1.0f, -dfdz);
        N.normalize();
        return N;
    }
    // génération procédurale maillage eau
    Mesh generateWaterMesh(float width, float depth, int subdivisions)
    {
        Mesh water;
        water.vertices.resize((subdivisions + 1) * (subdivisions + 1));
        water.triangles.resize(subdivisions * subdivisions * 2);

        float stepX = width / subdivisions;
        float stepZ = depth / subdivisions;
        float startX = -width / 2.0f;
        float startZ = -depth / 2.0f;

        // sommets
        for (int z = 0; z <= subdivisions; ++z)
        {
            for (int x = 0; x <= subdivisions; ++x)
            {
                float px = startX + x * stepX;
                float pz = startZ + z * stepZ;

                // On récupère la hauteur et la normale mathématiques
                float py = getWaterHeight(px, pz);
                Vec3 n = getWaterNormal(px, pz);

                int index = z * (subdivisions + 1) + x;
                water.vertices[index].position = Vec3(px, py, pz);
                water.vertices[index].normal = n;
                // UVs pour le mapping si besoin plus tard
                water.vertices[index].u = (float)x / subdivisions;
                water.vertices[index].v = (float)z / subdivisions;
            }
        }

        // triangles
        int t = 0;
        for (int z = 0; z < subdivisions; ++z)
        {
            for (int x = 0; x < subdivisions; ++x)
            {
                int row1 = z * (subdivisions + 1);
                int row2 = (z + 1) * (subdivisions + 1);

                // Triangle 1
                water.triangles[t].v[0] = row1 + x;
                water.triangles[t].v[1] = row2 + x;
                water.triangles[t].v[2] = row1 + x + 1;
                t++;

                // Triangle 2
                water.triangles[t].v[0] = row1 + x + 1;
                water.triangles[t].v[1] = row2 + x;
                water.triangles[t].v[2] = row2 + x + 1;
                t++;
            }
        }

        water.build_arrays();
        return water;
    }

    // scène sous-marine complète
    void setup_underwater_scene()
    {
        isUnderwaterScene = true;
        meshes.clear();
        spheres.clear();
        squares.clear();
        lights.clear();

        // 1. LA LUMIÈRE (Le soleil)
        // Elle doit être AU-DESSUS de l'eau, petite et puissante pour des caustiques nettes.
        {
            lights.resize(lights.size() + 1);
            Light &light = lights.back();
            light.pos = Vec3(0.0, 1.9f, 0.0); // Très haut

            // Petite taille = ombres et caustiques plus nettes
            light.u = Vec3(0.001f, 0.f, 0.f);
            light.v = Vec3(0.f, 0.f, 0.001f);

            light.powerCorrection = 8.f;
            light.type = LightType_Spherical;
            light.material = Vec3(1, 1, 1);
            light.isInCamSpace = false;
            light.samples = 64; // Plus d'échantillons pour la qualité
        }

        // 2. SURFACE DE L'EAU (MAILLAGE ONDULÉ)
        {
            meshes.resize(meshes.size() + 1); // On ajoute un Mesh, pas un Square
            Mesh &waterMesh = meshes.back();
            waterMesh = generateWaterMesh(5.2f, 5.2f, 300);
            waterMesh.translate(Vec3(0., 0.0f, 0.)); // Hauteur de l'eau
            waterMesh.build_arrays();
            waterMesh.material.type = Material_Water;
            waterMesh.material.diffuse_material = Vec3(0.8f, 0.9f, 0.95f);
            waterMesh.material.specular_material = Vec3(1.f, 1.f, 1.f);
            waterMesh.material.shininess = 60;
            waterMesh.material.transparency = 0.90f;
            waterMesh.material.index_medium = 1.33f;
        }

        Vec3 deepBlueColor(0.1f, 0.2f, 0.4f);
        Vec3 sandColor(0.6f, 0.55f, 0.45f);

        { // Floor (Sable)
            squares.resize(squares.size() + 1);
            Square &s = squares.back();
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.translate(Vec3(0., 0., -2.));
            s.scale(Vec3(2., 2., 1.));
            s.rotate_x(-90);
            s.build_arrays();
            s.material.diffuse_material = sandColor;
            s.material.specular_material = Vec3(0., 0., 0.); // Le sable est mat
            s.material.shininess = 10;
        }

        { // Back Wall
            squares.resize(squares.size() + 1);
            Square &s = squares.back();
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.scale(Vec3(2., 2., 1.));
            s.translate(Vec3(0., 0., -2.));
            s.build_arrays();
            s.material.diffuse_material = deepBlueColor;
            s.material.specular_material = Vec3(0.0f, 0.0f, 0.0f);
            s.material.shininess = 10;
        }

        { // Front Wall
            squares.resize(squares.size() + 1);
            Square &s = squares[squares.size() - 1];
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.translate(Vec3(0., 0., -2.));
            s.scale(Vec3(2., 2., 1.));
            s.rotate_y(180);
            s.build_arrays();
            s.material.diffuse_material = deepBlueColor;
            s.material.specular_material = Vec3(0.0f, 0.0f, 0.0f);
            s.material.shininess = 10;
        }

        { // Left Wall
            squares.resize(squares.size() + 1);
            Square &s = squares.back();
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.scale(Vec3(2., 2., 1.));
            s.translate(Vec3(0., 0., -2.));
            s.rotate_y(90);
            s.build_arrays();
            s.material.diffuse_material = deepBlueColor;
            s.material.specular_material = Vec3(0.0f, 0.0f, 0.0f);
            s.material.shininess = 10;
        }

        { // Right Wall
            squares.resize(squares.size() + 1);
            Square &s = squares.back();
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.translate(Vec3(0., 0., -2.));
            s.scale(Vec3(2., 2., 1.));
            s.rotate_y(-90);
            s.build_arrays();
            s.material.diffuse_material = deepBlueColor;
            s.material.specular_material = Vec3(0.0f, 0.0f, 0.0f);
            s.material.shininess = 10;
        }

        { // Ceiling (Blanc pour rebondir un peu de lumière ambiante)
            squares.resize(squares.size() + 1);
            Square &s = squares.back();
            s.setQuad(Vec3(-1., -1., 0.), Vec3(1., 0, 0.), Vec3(0., 1, 0.), 2., 2.);
            s.translate(Vec3(0., 0., -2.));
            s.scale(Vec3(2., 2., 1.));
            s.rotate_x(90);
            s.build_arrays();
            s.material.diffuse_material = Vec3(0.5f, 0.5f, 0.5f);
            s.material.specular_material = Vec3(0.0f, 0.0f, 0.0f);
            s.material.shininess = 0;
        }

        // 4. LE DRAGON (Au fond de l'eau)
        {
            meshes.resize(meshes.size() + 1);
            Mesh &mesh = meshes.back();
            mesh.loadOFF("data/xyzrgb_dragon_100k.off");
            mesh.centerAndScaleToUnit();
            mesh.scale(Vec3(1.5f, 1.5f, 1.5f));
            mesh.translate(Vec3(0.0f, -1.6f, -0.5f));
            mesh.rotate_y(220);
            mesh.build_arrays();
            mesh.material.type = Material_Diffuse_Blinn_Phong;
            mesh.material.diffuse_material = Vec3(0.9f, 0.8f, 0.6f); // Doré
            mesh.material.specular_material = Vec3(0.3f, 0.3f, 0.3f);
            mesh.material.shininess = 64.f;
        }

        // Rebuild KD-Tree
        kdDebugReset();
        kd.build(&squares, &spheres, &meshes, 1, 32);
        saveInitialLightState();
    }
};

#endif