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
    int maxDepth,
    Camera& camera,

    const std::vector<float3>& positions,
    const std::vector<float3>& normals,
    const std::vector<Triangle>& mesh,
    std::vector<float3>& image, // write your output to this array

    const BVH& bvh // Ignore this for now!
) {
    int pixelIdx = y * w + x;

    // To keep track of all the different light contributions that we have gotten
    float3 colors_sum = f3(0.0f, 0.0f, 0.0f);

    // ---------------------- Begin your code below --------------------------------




    

    image.at(pixelIdx) += colors_sum;
}