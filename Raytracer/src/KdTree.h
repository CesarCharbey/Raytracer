#pragma once
#include <vector>
#include <algorithm>
#include <limits>
#include <cfloat>
#include <cmath>

#include "Vec3.h"
#include "Ray.h"
#include "Sphere.h"
#include "Square.h"
#include "Triangle.h"
#include "Mesh.h"

#include <functional>
#include "DebugDraw.h"
#include <GL/glut.h>

// variables globales pour stats de debug
namespace kddebug
{
    static long long g_rayBoxTests = 0;
    static long long g_nodeVisits = 0;
    static long long g_leafVisits = 0;
    static long long g_primTests = 0;
}
// remise à zéro des compteurs
inline void kdDebugReset()
{
    kddebug::g_rayBoxTests = kddebug::g_nodeVisits = kddebug::g_leafVisits = kddebug::g_primTests = 0;
}

// Résultat d'intersection top-level
// type: 0=square (non utilisé), 1=sphere, 2=mesh
struct KdHit
{
    bool hit = false;
    float t = FLT_MAX;
    int type = -1;
    unsigned index = 0;

    RayTriangleIntersection triHit;
    RaySphereIntersection sphereHit;
    RaySquareIntersection squareHit;
};

// AABB + test rayon/boîte
// structure de boîte englobante alignée sur les axes
struct AABB
{
    Vec3 bmin, bmax;
    AABB()
        : bmin(std::numeric_limits<float>::infinity(),
               std::numeric_limits<float>::infinity(),
               std::numeric_limits<float>::infinity()),
          bmax(-std::numeric_limits<float>::infinity(),
               -std::numeric_limits<float>::infinity(),
               -std::numeric_limits<float>::infinity()) {}

    // étend la boîte pour inclure un point
    inline void expand(const Vec3 &p)
    {
        bmin[0] = std::min(bmin[0], p[0]);
        bmax[0] = std::max(bmax[0], p[0]);
        bmin[1] = std::min(bmin[1], p[1]);
        bmax[1] = std::max(bmax[1], p[1]);
        bmin[2] = std::min(bmin[2], p[2]);
        bmax[2] = std::max(bmax[2], p[2]);
    }
    // fusionne avec une autre boîte
    inline void expand(const AABB &b)
    {
        expand(b.bmin);
        expand(b.bmax);
    }
    // détermine l'axe le plus étendu
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

// test d'intersection rayon vs aabb (méthode des slabs)
inline bool rayAABB(const Vec3 &O, const Vec3 &D, const Vec3 &invD, const int sgn[3],
                    const AABB &B, float &tmin, float &tmax)
{
    using namespace kddebug;
    ++g_rayBoxTests;

    const float bounds[2][3] = {
        {B.bmin[0], B.bmin[1], B.bmin[2]},
        {B.bmax[0], B.bmax[1], B.bmax[2]}};

    // calcul des plans d'intersection x, y, z
    float t0 = (bounds[sgn[0]][0] - O[0]) * invD[0];
    float t1 = (bounds[1 - sgn[0]][0] - O[0]) * invD[0];
    float ty0 = (bounds[sgn[1]][1] - O[1]) * invD[1];
    float ty1 = (bounds[1 - sgn[1]][1] - O[1]) * invD[1];

    if ((t0 > ty1) || (ty0 > t1))
        return false;
    if (ty0 > t0)
        t0 = ty0;
    if (ty1 < t1)
        t1 = ty1;

    float tz0 = (bounds[sgn[2]][2] - O[2]) * invD[2];
    float tz1 = (bounds[1 - sgn[2]][2] - O[2]) * invD[2];

    if ((t0 > tz1) || (tz0 > t1))
        return false;
    if (tz0 > t0)
        t0 = tz0;
    if (tz1 < t1)
        t1 = tz1;

    tmin = t0;
    tmax = t1;
    return tmax >= std::max(0.0f, tmin);
}

// KD-tree top-level (indexe spheres + meshes)
// Les squares restent hors KD (déjà pré-testés côté Scene dans anuHitBefore())
class KdTree
{
public:
    KdTree() = default;

    inline bool empty() const { return nodes.empty(); }
    inline int nodeCount() const { return (int)nodes.size(); }
    inline int primCount() const { return (int)prims.size(); }

    // construction de l'arbre à partir des objets
    void build(std::vector<Square> *squares_,
               std::vector<Sphere> *spheres_,
               std::vector<Mesh> *meshes_,
               int minLeaf = 1, int maxDepth = 32)
    {
        squares = squares_;
        spheres = spheres_;
        meshes = meshes_;
        minPrimsPerLeaf = std::max(1, minLeaf);
        maxTreeDepth = std::max(1, maxDepth);

        prims.clear();
        indices.clear();
        indicesOut.clear();
        nodes.clear();

        // Spheres
        // ajout des sphères et calcul de leurs boîtes
        if (spheres)
        {
            for (int i = 0; i < (int)spheres->size(); ++i)
            {
                PrimRef pr;
                pr.type = 1;
                pr.index = i;
                const Sphere &s = (*spheres)[i];
                const Vec3 &c = s.m_center;
                float r = s.m_radius;
                AABB b;
                b.expand(c + Vec3(r, r, r));
                b.expand(c + Vec3(-r, -r, -r));
                pr.bounds = b;
                pr.centroid = b.centroid();
                prims.push_back(pr);
            }
        }
        // Meshes
        // ajout des maillages et calcul de leurs boîtes
        if (meshes)
        {
            for (int i = 0; i < (int)meshes->size(); ++i)
            {
                PrimRef pr;
                pr.type = 2;
                pr.index = i;
                AABB b;
                const Mesh &m = (*meshes)[i];
                if (!m.vertices.empty())
                {
                    for (const auto &v : m.vertices)
                        b.expand(v.position);
                }
                else if (!m.positions_array.empty())
                {
                    for (size_t k = 0; k + 2 < m.positions_array.size(); k += 3)
                        b.expand(Vec3(m.positions_array[k + 0], m.positions_array[k + 1], m.positions_array[k + 2]));
                }
                pr.bounds = b;
                pr.centroid = b.centroid();
                prims.push_back(pr);
            }
        }

        indices.resize(prims.size());
        for (int i = 0; i < (int)indices.size(); ++i)
            indices[i] = i;

        if (!prims.empty())
            buildRec(0, (int)prims.size(), 0);
    }

    // Intersection avec cutoff tMax (élagage agressif côté Scene)
    // parcours itératif de l'arbre
    KdHit intersect(const Ray &ray, float tMax) const
    {
        KdHit best;
        if (nodes.empty())
            return best;

        const Vec3 O = ray.origin();
        const Vec3 D = ray.direction();
        // inverse de la direction pour optimisation aabb
        const Vec3 invD(1.f / (D[0] != 0.f ? D[0] : 1e-30f),
                        1.f / (D[1] != 0.f ? D[1] : 1e-30f),
                        1.f / (D[2] != 0.f ? D[2] : 1e-30f));
        int sgn[3] = {invD[0] < 0.f, invD[1] < 0.f, invD[2] < 0.f};

        float tminRoot, tmaxRoot;
        if (!rayAABB(O, D, invD, sgn, nodes[0].box, tminRoot, tmaxRoot))
            return best;

        struct Task
        {
            int node;
            float tmin, tmax;
        };
        Task stack[128];
        int sp = 0;
        stack[sp++] = {0, tminRoot, tmaxRoot};

        using namespace kddebug;

        while (sp)
        {
            Task it = stack[--sp];
            const Node &N = nodes[it.node];
            ++g_nodeVisits;

            if (N.isLeaf())
            {
                ++g_leafVisits;
                // test d'intersection avec toutes les primitives de la feuille
                for (int i = 0; i < N.count; ++i)
                {
                    int pidx = indicesOut[N.start + i];
                    const PrimRef &pr = prims[pidx];
                    ++g_primTests;

                    if (pr.type == 1)
                    { // Sphere
                        const Sphere &s = (*spheres)[pr.index];
                        RaySphereIntersection h = s.intersect(ray);
                        if (h.intersectionExists && h.t > 1e-6f && h.t < best.t && h.t < tMax)
                        {
                            best.hit = true;
                            best.t = h.t;
                            best.type = 1;
                            best.index = pr.index;
                            best.sphereHit = h;
                        }
                    }
                    else
                    { // Mesh
                        const Mesh &m = (*meshes)[pr.index];
                        RayTriangleIntersection h = m.intersect(ray); // KD interne du mesh
                        if (h.intersectionExists && h.t > 1e-6f && h.t < best.t && h.t < tMax)
                        {
                            best.hit = true;
                            best.t = h.t;
                            best.type = 2;
                            best.index = pr.index;
                            best.triHit = h;
                        }
                    }
                }
                continue;
            }

            // Ordre near/far sans modifier le noeud
            float tminL, tmaxL, tminR2, tmaxR2;
            bool hitL = rayAABB(O, D, invD, sgn, nodes[N.left].box, tminL, tmaxL) && tminL <= std::min(best.t, tMax);
            bool hitR = rayAABB(O, D, invD, sgn, nodes[N.right].box, tminR2, tmaxR2) && tminR2 <= std::min(best.t, tMax);

            // empilement des enfants selon proximité
            if (hitL && hitR)
            {
                int nearId = N.left, farId = N.right;
                float tminNear = tminL, tmaxNear = tmaxL;
                float tminFar = tminR2, tmaxFar = tmaxR2;
                if (tminR2 < tminL)
                {
                    nearId = N.right;
                    farId = N.left;
                    tminNear = tminR2;
                    tmaxNear = tmaxR2;
                    tminFar = tminL;
                    tmaxFar = tmaxL;
                }
                stack[sp++] = {farId, tminFar, tmaxFar};
                stack[sp++] = {nearId, tminNear, tmaxNear};
            }
            else if (hitL)
            {
                stack[sp++] = {N.left, tminL, tmaxL};
            }
            else if (hitR)
            {
                stack[sp++] = {N.right, tminR2, tmaxR2};
            }
        }

        return best;
    }

    // Any-hit rapide pour ombres (early-exit)
    bool anyHitBefore(const Ray &ray, float tMax) const
    {
        const float eps = 1e-6f;

        // 1) Squares en bruteforce
        if (squares)
        {
            for (size_t i = 0; i < squares->size(); ++i)
            {
                RaySquareIntersection sq = (*squares)[i].intersect(ray);
                if (sq.intersectionExists && sq.t > eps && sq.t < tMax)
                    return true;
            }
        }

        if (nodes.empty())
            return false;

        const Vec3 O = ray.origin();
        const Vec3 D = ray.direction();
        const Vec3 invD(1.f / (D[0] != 0.f ? D[0] : 1e-30f),
                        1.f / (D[1] != 0.f ? D[1] : 1e-30f),
                        1.f / (D[2] != 0.f ? D[2] : 1e-30f));
        int sgn[3] = {invD[0] < 0.f, invD[1] < 0.f, invD[2] < 0.f};

        float tminRoot, tmaxRoot;
        if (!rayAABB(O, D, invD, sgn, nodes[0].box, tminRoot, tmaxRoot) || tminRoot > tMax)
            return false;

        struct Task
        {
            int node;
            float tmin, tmax;
        };
        Task stack[128];
        int sp = 0;
        stack[sp++] = {0, tminRoot, tmaxRoot};

        using namespace kddebug;

        while (sp)
        {
            Task it = stack[--sp];
            const Node &N = nodes[it.node];
            ++g_nodeVisits;

            if (N.isLeaf())
            {
                ++g_leafVisits;
                for (int i = 0; i < N.count; ++i)
                {
                    int pidx = indicesOut[N.start + i];
                    const PrimRef &pr = prims[pidx];
                    ++g_primTests;

                    if (pr.type == 1)
                    { // sphere
                        const Sphere &s = (*spheres)[pr.index];
                        RaySphereIntersection h = s.intersect(ray);
                        if (h.intersectionExists && h.t > eps && h.t < tMax)
                            return true; // retour immédiat si intersection trouvée
                    }
                    else
                    { // mesh
                        const Mesh &m = (*meshes)[pr.index];
                        // ultra-rapide: early-exit dans le KD interne du mesh
                        if (m.anyHitBefore_kdtree(ray, tMax))
                            return true;
                    }
                }
                continue;
            }

            float tminL, tmaxL, tminR2, tmaxR2;
            bool hitL = rayAABB(O, D, invD, sgn, nodes[N.left].box, tminL, tmaxL) && tminL < tMax;
            bool hitR = rayAABB(O, D, invD, sgn, nodes[N.right].box, tminR2, tmaxR2) && tminR2 < tMax;

            if (hitL && hitR)
            {
                int nearId = N.left, farId = N.right;
                float tminNear = tminL, tmaxNear = tmaxL;
                float tminFar = tminR2, tmaxFar = tmaxR2;
                if (tminR2 < tminL)
                {
                    nearId = N.right;
                    farId = N.left;
                    tminNear = tminR2;
                    tmaxNear = tmaxR2;
                    tminFar = tminL;
                    tmaxFar = tmaxL;
                }
                stack[sp++] = {farId, tminFar, tmaxFar};
                stack[sp++] = {nearId, tminNear, tmaxNear};
            }
            else if (hitL)
            {
                stack[sp++] = {N.left, tminL, tmaxL};
            }
            else if (hitR)
            {
                stack[sp++] = {N.right, tminR2, tmaxR2};
            }
        }
        return false;
    }

    // DEBUG KD global (objets). maxDepth < 0 => tout; leavesOnly => seulement feuilles
    void debugDraw(int maxDepth = -1, bool leavesOnly = false) const
    {
        if (nodes.empty())
            return;

        std::function<void(int, int)> dfs = [&](int idx, int depth)
        {
            const Node &N = nodes[idx];
            const bool drawThis = (maxDepth < 0 || depth <= maxDepth) && (!leavesOnly || N.isLeaf());

            if (drawThis)
            {
                if (N.isLeaf())
                    glColor3f(0.9f, 0.4f, 0.9f); // feuilles: violet
                else
                    glColor3f(0.2f, 0.6f, 1.0f); // internes: bleu
                drawAABBWire(N.box.bmin, N.box.bmax);
            }
            if (!N.isLeaf() && (maxDepth < 0 || depth < maxDepth))
            {
                dfs(N.left, depth + 1);
                dfs(N.right, depth + 1);
            }
        };

        glDisable(GL_LIGHTING);
        glLineWidth(1.0f);
        dfs(0, 0);
        glEnable(GL_LIGHTING);
    }

private:
    struct PrimRef // référence de primitives
    {
        int type = -1;  // 1=sphere, 2=mesh
        int index = -1; // index dans le conteneur d’origine
        AABB bounds;
        Vec3 centroid;
    };

    struct Node
    {
        AABB box;
        int left = -1;
        int right = -1;
        int start = 0; // offset dans indicesOut
        int count = 0; // >0 => feuille
        inline bool isLeaf() const { return count > 0; }
    };

    // construction récursive interne
    int buildRec(int begin, int end, int depth)
    {
        Node n;
        for (int i = begin; i < end; ++i)
            n.box.expand(prims[indices[i]].bounds);

        const int NB = end - begin;
        const int LEAF_SIZE = std::max(1, minPrimsPerLeaf);
        const int MAX_DEPTH = std::max(1, maxTreeDepth);

        // condition d'arrêt (feuille)
        if (NB <= LEAF_SIZE || depth >= MAX_DEPTH)
        {
            int id = (int)nodes.size();
            n.start = (int)indicesOut.size();
            n.count = NB;
            for (int i = begin; i < end; ++i)
                indicesOut.push_back(indices[i]);
            nodes.push_back(n);
            return id;
        }

        // découpe selon l'axe le plus long
        int axis = n.box.longestAxis();
        int mid = begin + NB / 2;
        std::nth_element(indices.begin() + begin, indices.begin() + mid, indices.begin() + end,
                         [&](int a, int b)
                         {
                             return prims[a].centroid[axis] < prims[b].centroid[axis];
                         });

        if (mid == begin || mid == end)
        { // dégénéré -> feuille
            int id = (int)nodes.size();
            n.start = (int)indicesOut.size();
            n.count = NB;
            for (int i = begin; i < end; ++i)
                indicesOut.push_back(indices[i]);
            nodes.push_back(n);
            return id;
        }

        int me = (int)nodes.size();
        nodes.push_back(n); // placeholder

        int L = buildRec(begin, mid, depth + 1);
        int R = buildRec(mid, end, depth + 1);

        nodes[me].left = L;
        nodes[me].right = R;
        nodes[me].box = n.box;
        return me;
    }

private:
    // Données KD
    std::vector<PrimRef> prims;
    std::vector<int> indices;    // temporaire (construction)
    std::vector<int> indicesOut; // compact pour feuilles
    std::vector<Node> nodes;

    // Paramètres
    int minPrimsPerLeaf = 1;
    int maxTreeDepth = 32;

    // Pointeurs vers la scène
    std::vector<Square> *squares = nullptr;
    std::vector<Sphere> *spheres = nullptr;
    std::vector<Mesh> *meshes = nullptr;
};