#pragma once

#include "utils.h"
#include <random>

/** * A function that handles diffuse (Lambertian) surface scattering by generating a random, 
 * cosine-weighted reflection direction over the hemisphere and calculating its math weights.
 * * Returns: nothing (void), but fills out the three output references passed into it.
 * * Takes in:
 * normal        : the surface normal at the hit point (automatically flipped if back-facing)
 * color         : the material color used to determine surface reflectivity
 * inDirection   : the world-space direction vector of the incoming ray hitting the surface
 * out_direction : output reference populated with the newly sampled world-space bounce direction
 * bsdf_val      : output reference populated with the evaluated BSDF color contribution (color / PI)
 * pdf           : output reference populated with the probability density function value of the choice
 * */
void inline sampleBSDF(
    float3 normal,
    float3 color,
    float3 inDirection, // world space

    float3& out_direction, // world space
    float3& bsdf_val,
    float& pdf
) {
    // -------------------- DONT TOUCH THIS ---------------------
    if (dot(inDirection, normal) > 0.0f) {
        normal = -normal;
    }

    float3 T;
    if (std::abs(normal.x) > 0.99f) {
        T = normalize(cross(normal, f3(0.0f, 1.0f, 0.0f)));
    } else {
        T = normalize(cross(normal, f3(1.0f, 0.0f, 0.0f)));
    }
    float3 B = cross(normal, T);

    float r1 = randomFloat(); 
    float r2 = randomFloat();

    float z = std::sqrt(r2);
    float r = std::sqrt(1.0f - r2);
    float phi = 2.0f * PI * r1;

    float x = r * std::cos(phi);
    float y = r * std::sin(phi);

    out_direction = normalize(T * x + B * y + normal * z);
    bsdf_val = color / PI; 
    pdf = z / PI;
}