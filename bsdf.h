#pragma once

#include "utils.h"
#include <random>

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