#pragma once

#include "stb_image_write.h"
#include "utils.h"
#include <vector>


static const std::array<float3, 3> aces_input_matrix =
{
    f3(0.59719f, 0.35458f, 0.04823f),
    f3(0.07600f, 0.90834f, 0.01566f),
    f3(0.02840f, 0.13383f, 0.83777f)
};

static const std::array<float3, 3> aces_output_matrix =
{
    f3( 1.60475f, -0.53108f, -0.07367f),
    f3(-0.10208f,  1.10813f, -0.00605f),
    f3(-0.00327f, -0.07276f,  1.07602f)
};

inline float3 mul(const std::array<float3, 3>& m, const float3& v)
{
    float x = m[0].x * v.x + m[0].y * v.y + m[0].z * v.z;
    float y = m[1].x * v.x + m[1].y * v.y + m[1].z * v.z;
    float z = m[2].x * v.x + m[2].y * v.y + m[2].z * v.z;
    return f3(x, y, z);
}

inline float3 rtt_and_odt_fit(float3 v)
{
    float3 a = v * (v + f3(0.0245786f)) - f3(0.000090537f);
    float3 b = v * (0.983729f * v + f3(0.4329510f)) + f3(0.238081f);
    return a / b;
}

inline float3 aces_fitted(float3 c)
{
    c = mul(aces_input_matrix, c);
    c = rtt_and_odt_fit(c);
    return mul(aces_output_matrix, c);
}

inline float3 toneMap(float3 color)
{
    const float A = 2.51f;
    const float B = 0.03f;
    const float C = 2.43f;
    const float D = 0.59f;
    const float E = 0.14f;

    return clampf3((color * (A * color + f3(B))) / (color * (C * color + f3(D)) + f3(E)), 0.0f, 1.0f);
}

inline float3 gammaCorrect(float3 c)
{
    float invGamma = 1.0f / 2.2f;
    return f3(
        powf(c.x, invGamma),
        powf(c.y, invGamma),
        powf(c.z, invGamma)
    );
}

inline void postProcessImage(const std::vector<float3>& image, std::vector<unsigned char>& out, int w, int h, int sample)
{
    std::vector<unsigned char> processed;

    #pragma omp parallel for schedule(static)
    for (int i = 0; i < w * h; i++)
    {   
        float3 averaged_color = image.at(i) / (float)(sample + 1);
    
        float3 final_color = gammaCorrect(aces_fitted(averaged_color));
        
        int byteIdx = i * 3;

        out[byteIdx + 0] = static_cast<unsigned char>(clamp(255.99f * final_color.x, 0.0f, 255.0f)); 
        out[byteIdx + 1] = static_cast<unsigned char>(clamp(255.99f * final_color.y, 0.0f, 255.0f)); 
        out[byteIdx + 2] = static_cast<unsigned char>(clamp(255.99f * final_color.z, 0.0f, 255.0f));
    }
}

inline void saveImage(const std::vector<unsigned char>& imageData, int w, int h, std::string name) {
    stbi_flip_vertically_on_write(true);
    int stride_in_bytes = w * 3;
    int success = stbi_write_png(name.c_str(), w, h, 3, imageData.data(), stride_in_bytes);

    if (!success) {
        std::cout << "Failed to write image.\n";
    }
}