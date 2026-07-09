#include "settings.h"
#include "utils.h"
#include "camera.h"
#include "render.h"
#include "bsdf.h"
#include "intersections.h"
#include <vector>

void renderPixel(
    int sampleNumber,
    int x, int y,
    int w, int h,
    Camera& camera,

    const std::vector<float3>& positions,
    const std::vector<float3>& normals,
    const std::vector<Triangle>& mesh,
    std::vector<float3>& image
) {
    // TODO: Make your path tracer!
}