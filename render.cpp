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
    int pixelIdx = y * w + x;

    // To keep track of all the different light contributions that we have gotten
    float3 colors_sum = f3(0.0f, 0.0f, 0.0f);

    // A value that encodes the effects of bouncing from the current point, 
    // all the way backwards to the camera. It is used by multiplying with the
    // emission of lights, so it starts at 1
    float3 throughput = f3(1.0f, 1.0f, 1.0f);

    Ray r = camera.generateCameraRay(x, y);

    for (int depth = 0; depth < BOUNCE_DEPTH; depth++) {
        Intersection intersect = traceClosestHit(r, positions, normals, mesh);

        if (!intersect.isValid) {
            break;
        }

        float3 position;
        float3 normal;
        float3 emission;
        float3 color;

        fetchGeometry(
            intersect,
            positions,
            normals,
            mesh,

            position,
            normal, 
            color,
            emission
        );

        if (lengthSquared(emission) > 0.0f) {
            //printf("emission %f %f %f\n", emission.x, emission.y, emission.z);
            colors_sum += emission * throughput;
        }

        float3 bsdf_value;
        float3 newDirection;
        float pdf;

        sampleBSDF(
            normal,
            color,
            r.direction,

            newDirection,
            bsdf_value,
            pdf
        );

        //printf("bsdf: %f %f %f pdf: %f\n", bsdf_value.x, bsdf_value.y, bsdf_value.z, pdf);

        if (pdf <= 1E-6) {
            break;
        }

        float surface_cosine = std::abs(dot(newDirection, normal));

        throughput *= bsdf_value * surface_cosine / pdf;

        r.direction = newDirection;
        r.origin = position + newDirection * 0.001f;
    }

    image.at(pixelIdx) += colors_sum;
    if (length(colors_sum) > 0.0f) {
        //printf("colors_sum is %f %f %f", colors_sum.x, colors_sum.y, colors_sum.z);
    }
}