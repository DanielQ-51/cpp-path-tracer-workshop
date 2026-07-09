#pragma once

#include "utils.h"

inline float3 rotateX(const float3& v, float angle)
{
    float c = cosf(angle), s = sinf(angle);
    return f3(
        v.x,
        v.y * c - v.z * s,
        v.y * s + v.z * c
    );
}

inline float3 rotateY(const float3& v, float angle)
{
    float c = cosf(angle), s = sinf(angle);
    return f3(
        v.x * c + v.z * s,
        v.y,
        -v.x * s + v.z * c
    );
}

inline float3 rotateZ(const float3& v, float angle)
{
    float c = cosf(angle), s = sinf(angle);
    return f3(
        v.x * c - v.y * s,
        v.x * s + v.y * c,
        v.z
    );
}

struct Camera
{
    float3 cameraOrigin;
    //float3 direction = f4(0.0f, 0.0f, -1.0f);
    int w;
    int h;

    float xRot; // in radians
    float yRot; // in radians
    float zRot; // in radians

    float aperture;
    float focalDist;
    //float imagePlaneDist; // for FOV
    float fovScale;

    float antiAliasJitterDist;

    float3 forward;
    float3 right;
    float3 up;

    static Camera Pinhole(const float3& cameraOrigin, int w, int h, float xR, float yR, float zR, float FOV, float aajitter = 0.0f)
    {
        Camera c;

        c.w = w;
        c.h = h;

        c.cameraOrigin = cameraOrigin;
        c.fovScale = tanf((FOV * 0.5f) * (3.141592f / 180.0f));
        c.xRot = xR * (3.14159265f / 180.0f);
        c.yRot = yR * (3.14159265f / 180.0f);
        c.zRot = zR * (3.14159265f / 180.0f);

        c.aperture = 0.000001f;
        c.focalDist = 1.0f/FOV;

        c.antiAliasJitterDist = aajitter;

        c.preCompute();

        return c;
    }

    static Camera NotPinhole(const float3& cameraOrigin, int w, int h, float xR, float yR, float zR, float FOV, float aperture, float focalDist, float aajitter = 2.0f)
    {
        Camera c;

        c.w = w;
        c.h = h;

        c.cameraOrigin = cameraOrigin;
        c.fovScale = tanf((FOV * 0.5f) * (3.141592f / 180.0f));
        c.xRot = xR * (3.14159265f / 180.0f);
        c.yRot = yR * (3.14159265f / 180.0f);
        c.zRot = zR * (3.14159265f / 180.0f);

        c.aperture = aperture;
        c.focalDist = focalDist;

        c.antiAliasJitterDist = aajitter;

        c.preCompute();
        return c;
    }

    inline Ray generateCameraRay(int x, int y) const
    {
        Ray r;
        float aspect = (float)w / (float)h;

        float jitterX = 0.0f;
        float jitterY = 0.0f;
        if (antiAliasJitterDist != 0.0f) {
            jitterX = (randomFloat() - 0.5f) * antiAliasJitterDist;
            jitterY = (randomFloat() - 0.5f) * antiAliasJitterDist;
        }

        float u = (2.0f * ((x + jitterX) / (float)w) - 1.0f) * aspect * fovScale;
        float v = (2.0f * ((y + jitterY) / (float)h) - 1.0f) * fovScale;

        float3 focalPoint = cameraOrigin + (right * (u * focalDist)) + (up * (v * focalDist)) + (forward * focalDist);

        r.origin = cameraOrigin;
        r.direction = normalize(focalPoint - r.origin);

        return r;
    }

    inline void preCompute()
    {
        float3 localForward = f3(0.0f, 0.0f, -1.0f);

        float3 worldForward = rotateX(localForward, xRot);
        worldForward = rotateY(worldForward, yRot);
        worldForward = rotateZ(worldForward, zRot);

        forward = normalize(worldForward);

        float3 localRight = f3(1.0f, 0.0f, 0.0f);
        right = normalize(rotateZ(rotateY(rotateX(localRight, xRot), yRot), zRot));

        float3 localUp = f3(0.0f, 1.0f, 0.0f);
        up = normalize(rotateZ(rotateY(rotateX(localUp, xRot), yRot), zRot));
        
    }

    inline float3 getForwardVector() const
    {
        return forward;
    }

    inline float3 getRightVector() const
    {
        return right;
    }

    inline float3 getUpVector() const
    {
        return up;
    }
};