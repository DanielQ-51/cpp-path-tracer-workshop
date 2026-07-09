#pragma once

#include "utils.h"

struct Intersection {
    float barycentric_u;
    float barycentric_v;

    int triangleIndex;
    bool isValid;
};

/**
 *  Returns true if the ray hits the triangle, and false otherwise.
 */
inline bool triangleIntersect(
    const std::vector<float3>& positions,
    const Triangle& tri,
    const Ray& r, 
    float& barycentric_u,
    float& barycentric_v,
    float& intersect_distance
)
{
    float3 tria = positions.at(tri.aInd);
    float3 trib = positions.at(tri.bInd);
    float3 tric = positions.at(tri.cInd);
    float3 e1 = trib - tria;
    float3 e2 = tric - tria;

    float3 h = cross(r.direction, e2);
    float a = dot(h, e1);

    if (fabsf(a) < 1E-6) 
        return false;
    
    float f = 1.0/a;

    float3 s = r.origin-tria;
    float u = f * dot(s, h);
    float3 q = cross(s, e1);
    float v = f * dot(r.direction, q);
    float t = f * dot(e2, q);


    if (((u >= 0) && (v >= 0) && (u + v <= 1)) && t > 0.0f)
    {
        barycentric_u = u;
        barycentric_v = v;
        intersect_distance = t;
        return true;
    }
    else
    {
        return false;
    }
}

// -------------------------------------------- Use the functions below --------------------------------------------------

/** 
 *  A function that finds the closest surface that the input ray intersects with, and returns
 *  an Intersection object.
 * 
 *  Returns: an Intersection object consisting of:
 *      float barycentric_u: the first barycentric coordinate
 *      float barycentric_v: the second barycentric coordinate
 *      int   triangleIndex: the index of the intersected triangle in the mesh array
 *      bool  isValid      : whether or not the ray hit any triangle
 * 
 *  Takes in:
 *  r        : the ray that is being tested
 *  positions: an array of all the vertex positions
 *  normals  : an array of all the vertex normals
 *  mesh     : an array of all the triangles in the scene
 *  
 */
inline Intersection traceClosestHit (
    Ray r,
    const std::vector<float3>& positions,
    const std::vector<float3>& normals,
    const std::vector<Triangle>& mesh
) {
    Intersection closest_hit;
    closest_hit.isValid = false;
    closest_hit.triangleIndex = -1;

    float closest_t = 1e30f;

    for (size_t i = 0; i < mesh.size(); ++i) {
        float current_u = 0.0f;
        float current_v = 0.0f;
        float current_t = 0.0f;

        if (triangleIntersect(positions, mesh[i], r, current_u, current_v, current_t)) {
            
            if (current_t < closest_t) {
                closest_t = current_t;
                
                closest_hit.isValid = true;
                closest_hit.triangleIndex = static_cast<int>(i);
                closest_hit.barycentric_u = current_u;
                closest_hit.barycentric_v = current_v;
            }
        }
    }

    return closest_hit;
}


/** * A function that uses barycentric coordinates to interpolate the precise hit position 
 * and normal vector on a triangle, and retrieves its material properties.
 * * Returns: nothing (void), but fills out the four output references passed into it.
 * * Takes in:
 * intersect    : the active intersection object containing hit coordinates and indices
 * positions    : an array of all the vertex positions
 * normals      : an array of all the vertex normals
 * mesh         : an array of all the triangles in the scene
 * out_position : output reference populated with the calculated world-space hit point
 * out_normal   : output reference populated with the interpolated and normalized surface normal
 * out_color    : output reference populated with the diffuse color of the hit triangle
 * out_emission : output reference populated with the light emission values of the hit triangle
 * */
inline void fetchGeometry(
    const Intersection& intersect,
    const std::vector<float3>& positions,
    const std::vector<float3>& normals,
    const std::vector<Triangle>& mesh,
    
    float3& out_position,
    float3& out_normal,
    float3& out_color,
    float3& out_emission
) {
    if (!intersect.isValid || intersect.triangleIndex < 0) {
        return;
    }

    const Triangle& tri = mesh[intersect.triangleIndex];

    float u = intersect.barycentric_u;
    float v = intersect.barycentric_v;
    float w = 1.0f - u - v;

    float3 posA = positions[tri.aInd];
    float3 posB = positions[tri.bInd];
    float3 posC = positions[tri.cInd];
    out_position = posA * w + posB * u + posC * v;

    float3 normA = normals[tri.naInd];
    float3 normB = normals[tri.nbInd];
    float3 normC = normals[tri.ncInd];
    out_normal = normalize(normA * w + normB * u + normC * v);

    out_color = tri.color;
    out_emission = tri.emission;
}