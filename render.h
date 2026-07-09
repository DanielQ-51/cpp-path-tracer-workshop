#pragma once

#include <vector>
#include "utils.h"
#include "camera.h"
#include "softwareBVH.h"

void renderPixel(
    int sampleNumber,
    int x, int y,
    int w, int h,
    int maxDepth,
    Camera& camera,

    const std::vector<float3>& positions,
    const std::vector<float3>& normals,
    const std::vector<Triangle>& mesh,
    std::vector<float3>& image,

    const BVH& bvh
);