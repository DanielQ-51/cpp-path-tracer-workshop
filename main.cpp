
#define STB_IMAGE_WRITE_IMPLEMENTATION

#include "settings.h"
#include <iostream>
#include <vector>
#include <chrono>
#include "utils.h"
#include "render.h"
#include "camera.h"
#include "imageUtils.h"


int main() {
    const int w = WIDTH;
    const int h = HEIGHT;

    // ------------------------------- Setting up --------------------------------------------

    std::vector<unsigned char> outImageBuffer(w * h * 3);
    std::vector<float3> image(w * h);

    Camera camera = Camera::Pinhole(
        f3(CAMERA_X_POS, CAMERA_Y_POS, CAMERA_Z_POS), 
        w, h, 
        CAMERA_X_ROT, CAMERA_Y_ROT, CAMERA_Z_ROT, 
        30.0f, // FOV (degrees)
        1.0f // subpixel jitter (in terms of pixels)
    );

    std::vector<Triangle> mesh;
    std::vector<float3> vertex_positions;
    std::vector<float3> vertex_normals;
    
    // ----------------------------------------------------------------------------------------------------------------------
    //                             Use this section to customize your scene with models exported from Maya
    // ----------------------------------------------------------------------------------------------------------------------
    
    readObjSimple("meshes/cornell_box_no_walls.obj", vertex_positions, vertex_normals, mesh, f3(0.9f, 0.9f, 0.9f), f3(0.0f));
    readObjSimple("meshes/cornell_box_left_wall.obj", vertex_positions, vertex_normals, mesh, f3(1.0f, 0.1f, 0.1f), f3(0.0f));
    readObjSimple("meshes/cornell_box_right_wall.obj", vertex_positions, vertex_normals, mesh, f3(0.1f, 1.0f, 0.1f), f3(0.0f));
    readObjSimple("meshes/cornell_box_left_box.obj", vertex_positions, vertex_normals, mesh, f3(0.8f, 0.8f, 0.8f), f3(0.0f));
    readObjSimple("meshes/cornell_box_right_box.obj", vertex_positions, vertex_normals, mesh, f3(0.8f, 0.8f, 0.8f), f3(0.0f));

    readObjSimple("meshes/cornell_box_large_light.obj", vertex_positions, vertex_normals, mesh, f3(0.8f, 0.8f, 0.8f), f3(18.0f, 18.0f, 12.0f));
    //readObjSimple("meshes/cornell_box_small_light.obj", vertex_positions, vertex_normals, mesh, f3(0.8f, 0.8f, 0.8f), f3(240.0f, 240.0f, 160.0f));

    // ----------------------------------------------------------------------------------------------------------------------
    
    printf("The scene has %d triangles\n", mesh.size());
    auto renderStartTime = std::chrono::steady_clock::now();

    printf("Starting Render\n");
    for (int sample = 0; sample < NUM_SAMPLE; sample++) {
        
        #pragma omp parallel for collapse(2) schedule(dynamic, 16)
        for (int x = 0; x < w; x++) {
            for (int y = 0; y < h; y++) {
                renderPixel(sample, x, y, w, h, BOUNCE_DEPTH, camera, vertex_positions, vertex_normals, mesh, image);
            }
        }

        printf("\rProgress: [Sample %d / %d]", sample + 1, NUM_SAMPLE);
        fflush(stdout);

        if ((sample + 1) % SAVE_INTERVAL == 0 || sample == NUM_SAMPLE - 1) {
            postProcessImage(image, outImageBuffer, w, h, sample);
            saveImage(outImageBuffer, w, h, "render.png");
        }
    }

    auto currentTime = std::chrono::steady_clock::now();
    std::chrono::duration<double, std::milli> elapsed = currentTime - renderStartTime;

    printf("\nRender finished in %f ms\n", elapsed.count());
    return 0;
}