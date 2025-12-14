#ifndef PHOTONKDTREE_H
#define PHOTONKDTREE_H

#include <vector>
#include <algorithm>
#include <cmath>
#include "Vec3.h"

struct Photon
{
    Vec3 position;
    Vec3 power;
    Vec3 incident;
};

// Structure interne d'un noeud de l'arbre
struct PhotonNode
{
    int photonIndex; // Index vers le tableau original de photons
    int axis;        // Axe de découpe (0=x, 1=y, 2=z)
    int left = -1;   // Indice enfant gauche
    int right = -1;  // Indice enfant droit
};

// Comparateur pour le tri median (std::nth_element)
struct PhotonComparator
{
    int axis;
    const std::vector<Photon> &photons;
    PhotonComparator(int ax, const std::vector<Photon> &p) : axis(ax), photons(p) {}

    bool operator()(int a, int b) const
    {
        return photons[a].position[axis] < photons[b].position[axis];
    }
};

class PhotonKdTree
{
private:
    std::vector<PhotonNode> nodes;
    std::vector<int> photonIndices;
    const std::vector<Photon> *photonsRef = nullptr;

public:
    void build(const std::vector<Photon> &photons)
    {
        photonsRef = &photons;
        nodes.clear();

        nodes.reserve(photons.size());
        photonIndices.resize(photons.size());

        for (size_t i = 0; i < photons.size(); ++i)
            photonIndices[i] = i;

        if (!photons.empty())
        {
            buildRec(0, (int)photons.size());
        }
    }

    void search(const Vec3 &p, float radius, std::vector<int> &results) const
    {
        if (nodes.empty())
            return;
        float radiusSq = radius * radius;
        searchRec(0, p, radius, radiusSq, results);
    }

private:
    int buildRec(int start, int end)
    {
        if (start >= end)
            return -1;

        // 1. Bounding Box
        Vec3 minB(FLT_MAX, FLT_MAX, FLT_MAX);
        Vec3 maxB(-FLT_MAX, -FLT_MAX, -FLT_MAX);

        for (int i = start; i < end; ++i)
        {
            const Vec3 &p = (*photonsRef)[photonIndices[i]].position;
            for (int k = 0; k < 3; ++k)
            {
                if (p[k] < minB[k])
                    minB[k] = p[k];
                if (p[k] > maxB[k])
                    maxB[k] = p[k];
            }
        }

        Vec3 diff = maxB - minB;
        int axis = 0;
        if (diff[1] > diff[0])
            axis = 1;
        if (diff[2] > diff[axis])
            axis = 2;

        // 2. Médiane
        int mid = (start + end) / 2;
        std::nth_element(
            photonIndices.begin() + start,
            photonIndices.begin() + mid,
            photonIndices.begin() + end,
            PhotonComparator(axis, *photonsRef));

        // 3. Noeud
        int nodeId = (int)nodes.size();
        nodes.push_back(PhotonNode());

        nodes[nodeId].photonIndex = photonIndices[mid];
        nodes[nodeId].axis = axis;

        // 4. Récursion
        int leftChild = buildRec(start, mid);
        int rightChild = buildRec(mid + 1, end);

        nodes[nodeId].left = leftChild;
        nodes[nodeId].right = rightChild;

        return nodeId;
    }

    void searchRec(int nodeId, const Vec3 &p, float radius, float radiusSq, std::vector<int> &results) const
    {
        if (nodeId == -1)
            return;

        const PhotonNode &node = nodes[nodeId];
        const Photon &ph = (*photonsRef)[node.photonIndex];

        // 1. Vérifier le photon
        float distSq = (ph.position - p).squareLength();
        if (distSq <= radiusSq)
        {
            results.push_back(node.photonIndex);
        }

        // 2. Descendre
        float distToPlane = p[node.axis] - ph.position[node.axis];

        if (distToPlane <= radius)
        {
            searchRec(node.left, p, radius, radiusSq, results);
        }
        if (distToPlane >= -radius)
        {
            searchRec(node.right, p, radius, radiusSq, results);
        }
    }
};

#endif