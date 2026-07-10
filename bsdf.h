#pragma once

#include "utils.h"

inline void createCoordinateSystem(const float3& N, float3& Nt, float3& Nb) {
    if (std::abs(N.x) > std::abs(N.y))
        Nt = normalize(f3(N.z, 0.0f, -N.x));
    else
        Nt = normalize(f3(0.0f, -N.z, N.y));
    Nb = cross(N, Nt);
}

inline float3 worldToLocal(const float3& v, const float3& Nt, const float3& Nb, const float3& N) {
    return f3(dot(v, Nt), dot(v, Nb), dot(v, N));
}

inline float3 localToWorld(const float3& v, const float3& Nt, const float3& Nb, const float3& N) {
    return f3(
        v.x * Nt.x + v.y * Nb.x + v.z * N.x,
        v.x * Nt.y + v.y * Nb.y + v.z * N.y,
        v.x * Nt.z + v.y * Nb.z + v.z * N.z
    );
}

// ---------------------- Microfacet GGX Math ----------------------
inline float D_GGX(const float3& h, float alpha) {
    float cosThetaH = h.z;
    if (cosThetaH <= 0.0f) return 0.0f;
    float alpha2 = alpha * alpha;
    float denom = cosThetaH * cosThetaH * (alpha2 - 1.0f) + 1.0f;
    return alpha2 / (PI * denom * denom);
}

inline float G1_GGX(const float3& v, float alpha) {
    float cosTheta = v.z;
    if (cosTheta <= 0.0f) return 0.0f;
    
    float alpha2 = alpha * alpha;
    float cosTheta2 = cosTheta * cosTheta;
    
    return (2.0f * cosTheta) / (cosTheta + std::sqrt(alpha2 + (1.0f - alpha2) * cosTheta2));
}

// Artist Friendly Schlick Approximation
inline float3 Fresnel_Schlick(float cosTheta, const float3& F0) {
    // Clamp to prevent floating-point errors from passing a negative base to pow()
    float c = std::clamp(cosTheta, 0.0f, 1.0f);
    
    float p5 = std::pow(1.0f - c, 5.0f);
    return F0 + (f3(1.0f, 1.0f, 1.0f) - F0) * p5;
}
// ---------------------- Core BSDF Sampler ----------------------

inline void sampleBSDF(
    int materialType,
    const float3& normal,
    const float3& color,
    const float3& inDirection, 
    float3& outDirection,
    float3& bsdf_value,
    float& pdf
) {
    // The ray comes FROM the camera TO the surface. Reverse it to point AWAY.
    float3 wi_world = normalize(inDirection * -1.0f);
    
    // Set up tangent space
    float3 Nt, Nb;
    createCoordinateSystem(normal, Nt, Nb);
    
    // Convert incoming ray to local space (Z is now straight up)
    float3 wi = worldToLocal(wi_world, Nt, Nb, normal);

    // Default catch for invalid angles
    if (wi.z <= 0.0f && materialType != TYPE_DIELECTRIC) {
        pdf = 0.0f;
        bsdf_value = f3(0.0f);
        return;
    }

    switch (materialType) {
        
        // ------------------ MATTE / LAMBERTIAN ------------------
        case TYPE_DIFFUSE: 
        {
            float r1 = randomFloat();
            float r2 = randomFloat();
            float phi = 2.0f * PI * r1;
            
            float x = std::cos(phi) * std::sqrt(r2);
            float y = std::sin(phi) * std::sqrt(r2);
            float z = std::sqrt(1.0f - r2);
            
            float3 wo = f3(x, y, z);
            outDirection = localToWorld(wo, Nt, Nb, normal);
            
            pdf = z / PI;
            bsdf_value = color / PI;
            break;
        }

        // ------------------ MICROFACET METALS ------------------
        case TYPE_ROUGH_METAL:
        case TYPE_SHINY_METAL:
        {
            float roughness = (materialType == TYPE_SHINY_METAL) ? 0.05f : 0.35f;
            float alpha = roughness * roughness;
            
            float r1 = randomFloat();
            float r2 = randomFloat();
            float phi = 2.0f * PI * r1;
            
            // Sample microfacet normal (h)
            float cosThetaH = std::sqrt((1.0f - r2) / (1.0f + (alpha * alpha - 1.0f) * r2));
            float sinThetaH = std::sqrt(std::max(0.0f, 1.0f - cosThetaH * cosThetaH));
            
            float3 h = f3(sinThetaH * std::cos(phi), sinThetaH * std::sin(phi), cosThetaH);
            
            // Reflect wi over the microfacet normal to get our outgoing direction
            float wiDotH = dot(wi, h);
            float3 wo = 2.0f * wiDotH * h - wi;
            
            if (wo.z <= 0.0f || wiDotH <= 0.0f) {
                pdf = 0.0f; bsdf_value = f3(0.0f);
                return;
            }
            
            outDirection = localToWorld(wo, Nt, Nb, normal);
            
            // Evaluate GGX components
            float D = D_GGX(h, alpha);
            float G = G1_GGX(wi, alpha) * G1_GGX(wo, alpha);
            float3 F = Fresnel_Schlick(wiDotH, color); // The magic! Color acts as F0 directly.
            
            pdf = (D * h.z) / (4.0f * wiDotH);
            bsdf_value = (F * D * G) / (4.0f * wi.z * wo.z);
            break;
        }

        // ------------------ GLASS / WATER ------------------
        case TYPE_DIELECTRIC:
        {
            float ior = 1.5f; // Hardcoded glass IOR
            bool isBackface = (wi.z < 0.0f);
            
            float etaI = isBackface ? ior : 1.0f;
            float etaT = isBackface ? 1.0f : ior;
            
            // Re-orient local Z if we are inside the object
            float cosI = std::abs(wi.z);
            float eta = etaI / etaT;
            float sin2_t = eta * eta * (1.0f - cosI * cosI);
            
            float R0 = (etaI - etaT) / (etaI + etaT);
            R0 = R0 * R0;
            float F = R0 + (1.0f - R0) * std::pow(1.0f - cosI, 5.0f);
            
            // Total Internal Reflection check
            if (sin2_t > 1.0f) F = 1.0f;

            // Russian Roulette: Randomly choose to Reflect OR Refract based on Fresnel Probability
            if (randomFloat() < F) {
                // Reflect
                float3 wo = f3(-wi.x, -wi.y, wi.z);
                outDirection = localToWorld(wo, Nt, Nb, normal);
                
                // Throughput cancels perfectly with Cosine when returning (1/cos)
                pdf = 1.0f; 
                bsdf_value = f3(1.0f / std::max(std::abs(wo.z), 0.0001f)); 
            } else {
                // Refract
                float cosT = std::sqrt(1.0f - sin2_t);
                float signZ = isBackface ? 1.0f : -1.0f;
                float3 wo = f3(-eta * wi.x, -eta * wi.y, signZ * cosT);
                
                outDirection = localToWorld(wo, Nt, Nb, normal);
                
                pdf = 1.0f;
                // Add adjoint radiance scaling (eta^2)
                bsdf_value = f3(1.0f / std::max(std::abs(wo.z), 0.0001f)) * (eta * eta);
            }
            break;
        }
    }
}