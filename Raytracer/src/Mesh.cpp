#include "Mesh.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <cfloat>
#include <cmath>

#include "DebugDraw.h"

void Mesh::loadOFF(const std::string &filename)
{
    std::ifstream in(filename.c_str());
    if (!in)
        exit(EXIT_FAILURE);
    std::string offString;
    unsigned int sizeV, sizeT, tmp;
    // lecture en-tête off : mot-clé, nb sommets, nb triangles, zéro
    in >> offString >> sizeV >> sizeT >> tmp;
    vertices.resize(sizeV);
    triangles.resize(sizeT);
    // boucle lecture positions xyz
    for (unsigned int i = 0; i < sizeV; i++)
        in >> vertices[i].position;
    int s;
    // boucle lecture indices triangles
    for (unsigned int i = 0; i < sizeT; i++)
    {
        in >> s; // nombre de sommets par face (toujours 3 ici)
        for (unsigned int j = 0; j < 3; j++)
            in >> triangles[i].v[j];
    }
    in.close();
}

void Mesh::recomputeNormals()
{
    // remise à zéro des normales existantes
    for (unsigned int i = 0; i < vertices.size(); i++)
        vertices[i].normal = Vec3(0.0, 0.0, 0.0);

    // calcul de la normale géométrique de chaque triangle
    for (unsigned int i = 0; i < triangles.size(); i++)
    {
        // vecteurs des arêtes du triangle
        Vec3 e01 = vertices[triangles[i].v[1]].position - vertices[triangles[i].v[0]].position;
        Vec3 e02 = vertices[triangles[i].v[2]].position - vertices[triangles[i].v[0]].position;
        // produit vectoriel = vecteur orthogonal au plan
        Vec3 n = Vec3::cross(e01, e02);
        n.normalize();
        // accumulation de la normale sur les 3 sommets du triangle
        for (unsigned int j = 0; j < 3; j++)
            vertices[triangles[i].v[j]].normal += n;
    }
    // normalisation finale (moyenne) pour lissage gouraud
    for (unsigned int i = 0; i < vertices.size(); i++)
        vertices[i].normal.normalize();
}

void Mesh::centerAndScaleToUnit()
{
    Vec3 c(0, 0, 0);
    // cumul de toutes les positions
    for (unsigned int i = 0; i < vertices.size(); i++)
        c += vertices[i].position;
    // division par le nombre de sommets pour avoir le centre
    c /= vertices.size();

    float maxD = (vertices[0].position - c).length();
    // recherche du sommet le plus éloigné du centre
    for (unsigned int i = 0; i < vertices.size(); i++)
    {
        float m = (vertices[i].position - c).length();
        if (m > maxD)
            maxD = m;
    }
    // application du décalage et mise à l'échelle
    for (unsigned int i = 0; i < vertices.size(); i++)
        vertices[i].position = (vertices[i].position - c) / maxD;
}

// =======================
// KD interne par mesh
// =======================
void Mesh::construire_tri_accel()
{
    tri_accel.clear();
    tri_accel.resize(triangles.size());
    // pré-calcul des données géométriques pour accélérer l'intersection
    for (size_t i = 0; i < triangles.size(); ++i)
    {
        const MeshTriangle &T = triangles[i];
        // récupération des positions des sommets
        const Vec3 &A = vertices[T.v[0]].position;
        const Vec3 &B = vertices[T.v[1]].position;
        const Vec3 &C = vertices[T.v[2]].position;

        TriAccel ta;
        ta.p0 = A;
        ta.e1 = B - A; // vecteur arête 1
        ta.e2 = C - A; // vecteur arête 2

        // normale géométrique brute
        ta.Ng = Vec3::cross(ta.e1, ta.e2);
        if (ta.Ng.squareLength() > 0.0f)
            ta.Ng.normalize();

        // calcul boîte englobante du triangle (min/max sur xyz)
        ta.box = AABB();
        ta.box.expand(A);
        ta.box.expand(B);
        ta.box.expand(C);
        ta.center = ta.box.centroid(); // centre pour le tri spatial

        tri_accel[i] = ta;
    }
}

static inline bool rayBoxIntersect(const Vec3 &O, const Vec3 &D, const Vec3 &invD, const int sgn[3],
                                   const Mesh::AABB &B, float &tmin, float &tmax)
{
    // définition des plans de la boîte
    const float bounds[2][3] = {
        {B.bmin[0], B.bmin[1], B.bmin[2]},
        {B.bmax[0], B.bmax[1], B.bmax[2]}};

    // intersection avec plans x
    float t0 = (bounds[sgn[0]][0] - O[0]) * invD[0];
    float t1 = (bounds[1 - sgn[0]][0] - O[0]) * invD[0];
    // intersection avec plans y
    float ty0 = (bounds[sgn[1]][1] - O[1]) * invD[1];
    float ty1 = (bounds[1 - sgn[1]][1] - O[1]) * invD[1];

    // vérification chevauchement intervalles x et y
    if ((t0 > ty1) || (ty0 > t1))
        return false;
    // réduction intervalle valide
    if (ty0 > t0)
        t0 = ty0;
    if (ty1 < t1)
        t1 = ty1;

    // intersection avec plans z
    float tz0 = (bounds[sgn[2]][2] - O[2]) * invD[2];
    float tz1 = (bounds[1 - sgn[2]][2] - O[2]) * invD[2];

    // vérification chevauchement final avec z
    if ((t0 > tz1) || (tz0 > t1))
        return false;
    if (tz0 > t0)
        t0 = tz0;
    if (tz1 < t1)
        t1 = tz1;

    // mise à jour des bornes de l'intersection
    tmin = t0;
    tmax = t1;
    // vrai si l'intervalle est valide et positif
    return tmax >= std::max(0.0f, tmin);
}

bool Mesh::rayAABB(const Vec3 &O, const Vec3 &D, const Vec3 &invD, const int sgn[3],
                   const AABB &B, float &tmin, float &tmax)
{
    return rayBoxIntersect(O, D, invD, sgn, B, tmin, tmax);
}

// Construction récursive du KD
static int build_kd_rec(Mesh *self, std::vector<int> &idx, int begin, int end, int depth)
{
    Mesh::KdNode n;
    // calcul de la boîte englobante de tous les triangles concernés
    for (int i = begin; i < end; ++i)
        n.box.expand(self->tri_accel[idx[i]].box);

    const int NB = end - begin;
    const int LEAF_SIZE = 16;
    const int MAX_DEPTH = 28;

    // condition d'arrêt : peu de triangles ou profondeur max atteinte
    if (NB <= LEAF_SIZE || depth >= MAX_DEPTH)
    {
        n.start = (int)self->tri_index.size();
        n.count = NB; // marqueur feuille (>0)
        // copie des indices de triangles dans le tableau global compact
        for (int i = begin; i < end; ++i)
            self->tri_index.push_back(idx[i]);
        self->kd_nodes.push_back(n);
        return (int)self->kd_nodes.size() - 1;
    }

    // choix de l'axe de découpe (le plus étendu)
    const int axis = n.box.longestAxis();
    const int mid = begin + NB / 2;

    // tri partiel pour mettre les triangles de "gauche" au début
    std::nth_element(idx.begin() + begin, idx.begin() + mid, idx.begin() + end,
                     [&](int a, int b)
                     {
                         return self->tri_accel[a].center[axis] < self->tri_accel[b].center[axis];
                     });

    // cas dégénéré : impossible de séparer -> on fait une feuille
    if (mid == begin || mid == end)
    {
        n.start = (int)self->tri_index.size();
        n.count = NB;
        for (int i = begin; i < end; ++i)
            self->tri_index.push_back(idx[i]);
        self->kd_nodes.push_back(n);
        return (int)self->kd_nodes.size() - 1;
    }

    // création noeud interne et appels récursifs
    const int me = (int)self->kd_nodes.size();
    self->kd_nodes.push_back(n); // réservation place

    int L = build_kd_rec(self, idx, begin, mid, depth + 1);
    int R = build_kd_rec(self, idx, mid, end, depth + 1);

    // liaison enfants
    self->kd_nodes[me].left = L;
    self->kd_nodes[me].right = R;
    self->kd_nodes[me].box = n.box;
    return me;
}

void Mesh::construire_kdtree()
{
    kd_nodes.clear();
    tri_index.clear();
    if (tri_accel.empty())
        return;

    // tableau d'indices temporaire pour le tri
    std::vector<int> idx(tri_accel.size());
    for (int i = 0; i < (int)idx.size(); ++i)
        idx[i] = i;

    // lancement récursion racine
    build_kd_rec(this, idx, 0, (int)idx.size(), 0);
}

// Intersections kd (retourne la version complète avec barycentrique + normale lissée)
RayTriangleIntersection Mesh::intersect_kdtree(Ray const &ray) const
{
    RayTriangleIntersection best;
    best.intersectionExists = false;
    best.t = FLT_MAX;
    best.tIndex = 0;

    if (kd_nodes.empty())
    {
        // fallback bruteforce si pas de kd-tree construit
        for (size_t ti = 0; ti < triangles.size(); ++ti)
        {
            const MeshTriangle &triIndices = triangles[ti];
            // reconstruction triangle temporaire
            const Vec3 &A = vertices[triIndices.v[0]].position;
            const Vec3 &B = vertices[triIndices.v[1]].position;
            const Vec3 &C = vertices[triIndices.v[2]].position;
            Triangle T(A, B, C);
            RayTriangleIntersection inter = T.getIntersection(ray);
            if (inter.intersectionExists && inter.t < best.t)
                best = inter;
        }
        return best;
    }

    // pré-calculs pour traversée aabb
    const Vec3 O = ray.origin();
    const Vec3 D = ray.direction();
    const Vec3 invD(1.f / (D[0] != 0.f ? D[0] : 1e-30f),
                    1.f / (D[1] != 0.f ? D[1] : 1e-30f),
                    1.f / (D[2] != 0.f ? D[2] : 1e-30f));
    int sgn[3] = {invD[0] < 0.f, invD[1] < 0.f, invD[2] < 0.f};

    // test intersection racine kd-tree
    float tminR, tmaxR;
    if (!rayBoxIntersect(O, D, invD, sgn, kd_nodes[0].box, tminR, tmaxR))
        return best; // rayon rate la boîte globale

    // pile pour parcours itératif
    struct Task
    {
        int node;
        float tmin, tmax;
    };
    Task stack[128];
    int sp = 0;
    stack[sp++] = {0, tminR, tmaxR};

    // boucle principale de traversée
    while (sp)
    {
        Task it = stack[--sp];
        const KdNode &N = kd_nodes[it.node];

        // si feuille : tester tous les triangles contenus
        if (N.isLeaf())
        {
            for (int i = 0; i < N.count; ++i)
            {
                int f = tri_index[N.start + i];
                const TriAccel &T = tri_accel[f];

                // algorithme möller-trumbore
                Vec3 pvec = Vec3::cross(D, T.e2);
                float det = Vec3::dot(T.e1, pvec);
                // rayon parallèle au triangle
                if (std::fabs(det) < 1e-8f)
                    continue;
                float invDet = 1.0f / det;
                Vec3 tvec = O - T.p0;
                // coordonnée barycentrique u
                float u = Vec3::dot(tvec, pvec) * invDet;
                if (u < -1e-6f || u > 1.f + 1e-6f)
                    continue;
                Vec3 qvec = Vec3::cross(tvec, T.e1);
                // coordonnée barycentrique v
                float v = Vec3::dot(D, qvec) * invDet;
                if (v < -1e-6f || (u + v) > 1.f + 1e-6f)
                    continue;
                // distance t
                float t = Vec3::dot(T.e2, qvec) * invDet;

                // validation si t positif et plus proche que le meilleur actuel
                if (t > 1e-6f && t < best.t)
                {
                    best.intersectionExists = true;
                    best.t = t;
                    best.tIndex = f;
                    best.w1 = u;
                    best.w2 = v;
                    best.w0 = 1.f - u - v;
                    best.intersection = O + t * D;

                    // interpolation de la normale (shading lissé)
                    const MeshTriangle &tri = triangles[f];
                    const MeshVertex &v0 = vertices[tri.v[0]];
                    const MeshVertex &v1 = vertices[tri.v[1]];
                    const MeshVertex &v2 = vertices[tri.v[2]];
                    Vec3 nInterp = best.w0 * v0.normal + best.w1 * v1.normal + best.w2 * v2.normal;
                    if (nInterp.squareLength() > 0.0f)
                        nInterp.normalize();
                    best.normal = nInterp;
                }
            }
            continue; // fin traitement feuille, on dépile suite
        }

        // noeud interne : test intersection enfants
        float tminL = 0.0f, tmaxL = 0.0f, tminR2 = 0.0f, tmaxR2 = 0.0f;
        bool hitL = rayBoxIntersect(O, D, invD, sgn, kd_nodes[N.left].box, tminL, tmaxL);
        bool hitR = rayBoxIntersect(O, D, invD, sgn, kd_nodes[N.right].box, tminR2, tmaxR2);

        // logique de traversée : proche d'abord (front-to-back)
        if (hitL && hitR)
        {
            int nearId = N.left, farId = N.right;
            float tminNear = tminL, tmaxNear = tmaxL, tminFar = tminR2, tmaxFar = tmaxR2;
            // inversion si droite plus proche
            if (tminR2 < tminL)
            {
                nearId = N.right;
                farId = N.left;
                tminNear = tminR2;
                tmaxNear = tmaxR2;
                tminFar = tminL;
                tmaxFar = tmaxL;
            }
            // on empile le plus loin d'abord (traité en dernier)
            if (tminFar <= best.t)
                stack[sp++] = {farId, tminFar, tmaxFar};
            // on empile le plus proche ensuite (traité tout de suite)
            if (tminNear <= best.t)
                stack[sp++] = {nearId, tminNear, tmaxNear};
        }
        else if (hitL)
        {
            if (tminL <= best.t)
                stack[sp++] = {N.left, tminL, tmaxL};
        }
        else if (hitR)
        {
            if (tminR2 <= best.t)
                stack[sp++] = {N.right, tminR2, tmaxR2};
        }
    }

    return best;
}

// Any-hit rapide (pour ombres)
bool Mesh::anyHitBefore_kdtree(Ray const &ray, float tMax) const
{
    if (kd_nodes.empty())
    {
        // fallback bruteforce
        for (size_t ti = 0; ti < triangles.size(); ++ti)
        {
            const MeshTriangle &tri = triangles[ti];
            Triangle T(vertices[tri.v[0]].position, vertices[tri.v[1]].position, vertices[tri.v[2]].position);
            RayTriangleIntersection h = T.getIntersection(ray);
            if (h.intersectionExists && h.t > 1e-6f && h.t < tMax)
                return true;
        }
        return false;
    }

    const Vec3 O = ray.origin();
    const Vec3 D = ray.direction();
    const Vec3 invD(1.f / (D[0] != 0.f ? D[0] : 1e-30f),
                    1.f / (D[1] != 0.f ? D[1] : 1e-30f),
                    1.f / (D[2] != 0.f ? D[2] : 1e-30f));
    int sgn[3] = {invD[0] < 0.f, invD[1] < 0.f, invD[2] < 0.f};

    float tminR, tmaxR;
    // si on rate la boîte racine ou si c'est trop loin > tMax
    if (!rayBoxIntersect(O, D, invD, sgn, kd_nodes[0].box, tminR, tmaxR) || tminR > tMax)
        return false;

    struct Task
    {
        int node;
        float tmin, tmax;
    };
    Task stack[128];
    int sp = 0;
    stack[sp++] = {0, tminR, tmaxR};

    while (sp)
    {
        Task it = stack[--sp];
        const KdNode &N = kd_nodes[it.node];

        if (N.isLeaf())
        {
            for (int i = 0; i < N.count; ++i)
            {
                int f = tri_index[N.start + i];
                const TriAccel &T = tri_accel[f];

                // möller–trumbore optimisé pour test booléen
                Vec3 pvec = Vec3::cross(D, T.e2);
                float det = Vec3::dot(T.e1, pvec);
                if (std::fabs(det) < 1e-8f)
                    continue;
                float invDet = 1.0f / det;
                Vec3 tvec = O - T.p0;
                float u = Vec3::dot(tvec, pvec) * invDet;
                if (u < 0.f || u > 1.f)
                    continue;
                Vec3 qvec = Vec3::cross(tvec, T.e1);
                float v = Vec3::dot(D, qvec) * invDet;
                if (v < 0.f || (u + v) > 1.f)
                    continue;
                float t = Vec3::dot(T.e2, qvec) * invDet;
                // si intersection valide avant tMax, on sort immédiatement (ombre trouvée)
                if (t > 1e-6f && t < tMax)
                    return true;
            }
            continue;
        }

        // test enfants
        float tminL = 0.0f, tmaxL = 0.0f, tminR2 = 0.0f, tmaxR2 = 0.0f;
        bool hitL = rayBoxIntersect(O, D, invD, sgn, kd_nodes[N.left].box, tminL, tmaxL) && tminL < tMax;
        bool hitR = rayBoxIntersect(O, D, invD, sgn, kd_nodes[N.right].box, tminR2, tmaxR2) && tminR2 < tMax;

        // empilement ordonné
        if (hitL && hitR)
        {
            int nearId = N.left, farId = N.right;
            float tminNear = tminL, tmaxNear = tmaxL, tminFar = tminR2, tmaxFar = tmaxR2;
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

RayTriangleIntersection Mesh::intersect(Ray const &ray) const
{
    return intersect_kdtree(ray);
}

void Mesh::debugDrawKd(int maxDepth, bool leavesOnly) const
{
    if (kd_nodes.empty())
        return;

    // structure locale pour parcourir l'arbre récursivement
    struct Walker
    {
        const Mesh *self;
        int maxDepth;
        bool leavesOnly;

        void run(int idx, int depth) const
        {
            const KdNode &N = self->kd_nodes[idx];
            // décide si on dessine ce niveau
            const bool drawThis = (maxDepth < 0 || depth <= maxDepth) && (!leavesOnly || N.isLeaf());

            if (drawThis)
            {
                if (N.isLeaf())
                    glColor3f(1.f, 0.25f, 0.25f); // feuilles: rouge
                else
                    glColor3f(0.2f + 0.8f * (depth / 24.f), 0.8f, 0.2f); // internes : dégradé vert
                drawAABBWire(N.box.bmin, N.box.bmax);
            }

            if (!N.isLeaf() && (maxDepth < 0 || depth < maxDepth))
            {
                run(N.left, depth + 1);
                run(N.right, depth + 1);
            }
        }
    };

    Walker w = {this, maxDepth, leavesOnly};

    glDisable(GL_LIGHTING);
    glLineWidth(1.0f);
    w.run(0, 0);
    glEnable(GL_LIGHTING);
}