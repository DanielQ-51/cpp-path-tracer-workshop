#pragma once

#include "utils.h"
#include <vector>
#include <algorithm>
#include <cfloat>

struct BVHNode {
    float3 aabbMin;
    float3 aabbMax;
    int left;
    int right;
    int first;
    int primCount;

    BVHNode() {}
    BVHNode(float3 min, float3 max, int l, int r, int f, int ct)
        : aabbMin(min), aabbMax(max), left(l), right(r), first(f), primCount(ct) {}
};

// --- float3 BVH Math Helpers ---
inline float getAxis(const float3& v, int axis) {
    if (axis == 0) return v.x;
    if (axis == 1) return v.y;
    return v.z;
}

inline float3 minf3(const float3& a, const float3& b) {
    return f3(std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z));
}

inline float3 maxf3(const float3& a, const float3& b) {
    return f3(std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z));
}

inline float surfaceArea(const float3& minB, const float3& maxB) {
    float3 d = maxB - minB;
    // Clamp to 0 to prevent negative areas from degenerate triangles
    d.x = std::max(0.0f, d.x);
    d.y = std::max(0.0f, d.y);
    d.z = std::max(0.0f, d.z);
    return 2.0f * (d.x * d.y + d.y * d.z + d.z * d.x);
}
// -------------------------------

inline void computeInfoForBVH(
    const std::vector<float3>& positions,
    const std::vector<Triangle>& mesh,
    std::vector<float3>& centroids,
    std::vector<float3>& AABBmins,
    std::vector<float3>& AABBmaxes,
    std::vector<int>& originalIndices
) {
    for (int i = 0; i < mesh.size(); i++) {
        const Triangle& tri = mesh[i];
        float3 a = positions[tri.aInd];
        float3 b = positions[tri.bInd];
        float3 c = positions[tri.cInd];

        centroids.push_back((a + b + c) / 3.0f);

        // Compute AABB min
        float3 minPos = f3(
            std::min({a.x, b.x, c.x}),
            std::min({a.y, b.y, c.y}),
            std::min({a.z, b.z, c.z})
        ) - f3(0.000001f);
        AABBmins.push_back(minPos);

        // Compute AABB max
        float3 maxPos = f3(
            std::max({a.x, b.x, c.x}),
            std::max({a.y, b.y, c.y}),
            std::max({a.z, b.z, c.z})
        ) + f3(0.000001f);
        AABBmaxes.push_back(maxPos);

        originalIndices.push_back(i);
    }
}

inline int partitionPrimitives(std::vector<int>& indices, const std::vector<float3>& centroids, int start, int end, int axis, float splitPos)
{
    int mid = start;
    for (int i = start; i < end; i++) {
        if (getAxis(centroids[indices[i]], axis) < splitPos) {
            std::swap(indices[i], indices[mid]);
            mid++;
        }
    }
    return mid;
}

inline void SAH(
    std::vector<int>& indices, const std::vector<float3>& centroids, 
    const std::vector<float3>& AABBmins, const std::vector<float3>& AABBmaxes, 
    int start, int end, int axis, 
    float3 minBound, float3 maxBound, float& splitPos, float& minCost
) {
    const int numBuckets = 8;
    struct Bucket { float3 min, max; int count; };
    Bucket buckets[numBuckets];
    
    for (int i = 0; i < numBuckets; i++) {
        buckets[i].min = f3(FLT_MAX);
        buckets[i].max = f3(-FLT_MAX);
        buckets[i].count = 0;
    }

    float minC = getAxis(minBound, axis);
    float maxC = getAxis(maxBound, axis);
    float extent = maxC - minC;

    for (int i = start; i < end; i++) {
        int idx = indices[i];
        float c = getAxis(centroids[idx], axis);
        
        int b = 0;
        if (extent > 0.0f) {
            b = static_cast<int>(numBuckets * (c - minC) / extent);
            b = clamp(b, 0, numBuckets - 1);
        }
        
        buckets[b].count++;
        buckets[b].min = minf3(buckets[b].min, AABBmins[idx]);
        buckets[b].max = maxf3(buckets[b].max, AABBmaxes[idx]);
    }

    minCost = FLT_MAX;
    int bestSplit = -1;
    
    for (int i = 1; i < numBuckets; i++) {
        float3 leftMin = buckets[0].min;
        float3 leftMax = buckets[0].max;
        int leftCount = buckets[0].count;
        for (int j = 1; j < i; j++) {
            leftMin = minf3(leftMin, buckets[j].min);
            leftMax = maxf3(leftMax, buckets[j].max);
            leftCount += buckets[j].count;
        }

        float3 rightMin = buckets[i].min;
        float3 rightMax = buckets[i].max;
        int rightCount = buckets[i].count;
        for (int j = i; j < numBuckets; j++) {
            rightMin = minf3(rightMin, buckets[j].min);
            rightMax = maxf3(rightMax, buckets[j].max);
            rightCount += buckets[j].count;
        }

        float cost = 1.0f + (leftCount * surfaceArea(leftMin, leftMax) + rightCount * surfaceArea(rightMin, rightMax))
                           / surfaceArea(minBound, maxBound);
                           
        if (cost < minCost && leftCount > 0 && rightCount > 0) {
            minCost = cost;
            bestSplit = i;
        }
    }

    if (bestSplit == -1) {
        // Fallback to sorting by centroid midpoint if bucketing failed
        int mid = (start + end) / 2;
        std::nth_element(indices.begin() + start, indices.begin() + mid, indices.begin() + end,
            [&](int a, int b) { return getAxis(centroids[a], axis) < getAxis(centroids[b], axis); });
        
        splitPos = getAxis(centroids[indices[mid]], axis);
    } else {
        splitPos = minC + extent * (float(bestSplit) / float(numBuckets));
    }
}

struct BVH {
    std::vector<BVHNode> nodes; 
    std::vector<int> indices;
};

inline int buildBVH(
    BVH& bvh, const std::vector<float3>& centroids, 
    const std::vector<float3>& AABBmins, const std::vector<float3>& AABBmaxes, 
    int start, int end, int maxLeafSize
) {
    std::vector<BVHNode>& nodes = bvh.nodes; 
    std::vector<int>& indices = bvh.indices;

    int nodeIndex = nodes.size();
    nodes.push_back(BVHNode());

    float3 minBound = AABBmins[indices[start]];
    float3 maxBound = AABBmaxes[indices[start]];
    for (int i = start + 1; i < end; i++) {
        int idx = indices[i];
        minBound = minf3(minBound, AABBmins[idx]);
        maxBound = maxf3(maxBound, AABBmaxes[idx]);
    }
    
    nodes[nodeIndex].aabbMin = minBound;
    nodes[nodeIndex].aabbMax = maxBound;

    int primCount = end - start;

    if (primCount <= maxLeafSize) {
        nodes[nodeIndex].first = start;
        nodes[nodeIndex].primCount = primCount;
        nodes[nodeIndex].left = -1;
        nodes[nodeIndex].right = -1;
        return nodeIndex;
    }

    float cost = FLT_MAX;
    int axis = -1;
    float splitPos = 0.0f;

    // Loop through all 3 axes to find the lowest SAH cost
    for (int currentAxis = 0; currentAxis < 3; ++currentAxis) {
        float currentSplitPos;
        float currentCost;
        
        SAH(indices, centroids, AABBmins, AABBmaxes, start, end, currentAxis, 
            minBound, maxBound, currentSplitPos, currentCost);

        if (currentCost < cost) {
            cost = currentCost;
            axis = currentAxis;
            splitPos = currentSplitPos;
        }
    }

    int numLeft = 0;
    for (int i = start; i < end; i++) {
        if (getAxis(centroids[indices[i]], axis) < splitPos)
            numLeft++;
    }

    int mid;
    if (numLeft > 0 && numLeft < primCount) {
        mid = partitionPrimitives(indices, centroids, start, end, axis, splitPos);
    } else {
        // Fallback: If SAH split fails to partition, force a spatial midpoint split
        float sum = 0.0f;
        for (int i = start; i < end; i++) {
            sum += getAxis(centroids[indices[i]], axis);
        }
        splitPos = sum / primCount;
        
        numLeft = 0;
        for (int i = start; i < end; i++) {
            if (getAxis(centroids[indices[i]], axis) < splitPos)
                numLeft++;
        }
        
        if (numLeft > 0 && numLeft < primCount) {
            mid = partitionPrimitives(indices, centroids, start, end, axis, splitPos);
        } else {
            // Hard Fallback: Force a leaf if geometry is perfectly stacked/identical
            nodes[nodeIndex].first = start;
            nodes[nodeIndex].primCount = primCount;
            nodes[nodeIndex].left = -1;
            nodes[nodeIndex].right = -1;
            return nodeIndex;
        }
    }

    nodes[nodeIndex].left  = buildBVH(bvh, centroids, AABBmins, AABBmaxes, start, mid, maxLeafSize);
    nodes[nodeIndex].right = buildBVH(bvh, centroids, AABBmins, AABBmaxes, mid, end, maxLeafSize);

    nodes[nodeIndex].primCount = 0;
    nodes[nodeIndex].first = -1;
    
    return nodeIndex;
}

/**
 * Fast slab-based bounding box intersection test.
 */
inline bool aabbIntersect(
    const Ray& r, 
    const float3& minCorner, 
    const float3& maxCorner, 
    const float3& invDir, 
    float& tmin, 
    float& tmax
) {
    tmin = -1e30f; 
    tmax = 1e30f;  

    // X axis
    float tx1 = (minCorner.x - r.origin.x) * invDir.x;
    float tx2 = (maxCorner.x - r.origin.x) * invDir.x;
    tmin = std::max(tmin, std::min(tx1, tx2));
    tmax = std::min(tmax, std::max(tx1, tx2));

    // Y axis
    float ty1 = (minCorner.y - r.origin.y) * invDir.y;
    float ty2 = (maxCorner.y - r.origin.y) * invDir.y;
    tmin = std::max(tmin, std::min(ty1, ty2));
    tmax = std::min(tmax, std::max(ty1, ty2));

    // Z axis
    float tz1 = (minCorner.z - r.origin.z) * invDir.z;
    float tz2 = (maxCorner.z - r.origin.z) * invDir.z;
    tmin = std::max(tmin, std::min(tz1, tz2));
    tmax = std::min(tmax, std::max(tz1, tz2));
    
    return (tmax >= tmin) && (tmax > 0.0f);
}
