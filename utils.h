#pragma once

#include <cmath>
#include <array>
#include <vector>
#include <algorithm>
#include <string>
#include "stb_image_write.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <random>

constexpr float PI = 3.141592f;

struct float3
{
    float x, y, z, w;
};

inline float3 make_float3(float x, float y, float z)
{
  float3 t; t.x = x; t.y = y; t.z = z; return t;
}

inline float3 f3(float x, float y, float z) {
    return make_float3(x, y, z);
}

inline float3 f3() {return make_float3(0,0,0);}

inline float3 f3(float a) {return make_float3(a,a,a);}


inline float3 operator+(const float3 &a, const float3 &b) {
    return make_float3(a.x + b.x, a.y + b.y, a.z + b.z);
}

inline float3 operator-(const float3 &a, const float3 &b) {
    return make_float3(a.x - b.x, a.y - b.y, a.z - b.z);
}

inline float3 operator*(const float3 &a, float t) {
    return make_float3(a.x * t, a.y * t, a.z * t);
}

inline float3 operator*(float t, const float3 &a) {
    return a * t;
}

inline float3 operator/(const float3 &a, float t) {
    return make_float3(a.x / t, a.y / t, a.z / t);
}

inline float3& operator+=(float3 &a, const float3 &b) {
    a.x += b.x; a.y += b.y; a.z += b.z;
    return a;
}

inline float3& operator*=(float3 &a, float t) {
    a.x *= t; a.y *= t; a.z *= t;
    return a;
}

inline float3& operator*=(float3& a, const float3& b) {
    a.x *= b.x; a.y *= b.y; a.z *= b.z;
    return a;
}

inline  float3& operator/=(float3 &a, float t) {
    a.x /= t; a.y /= t; a.z /= t;
    return a;
}

inline  float3 operator*(const float3& a, const float3& b) {
    return make_float3(a.x*b.x, a.y*b.y, a.z*b.z);
}

inline  float3 operator/(const float3& a, const float3& b) {
    return make_float3(a.x/b.x, a.y/b.y, a.z/b.z);
}

inline  float3 operator-(const float3& v) {
    return make_float3(-v.x, -v.y, -v.z);
}

inline  float dot(const float3 &a, const float3 &b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

inline  float length(const float3 &v) {
    return std::sqrt(dot(v, v));
}

inline  float lengthSquared(const float3 &v) {
    return dot(v, v);
}

inline  float3 normalize(const float3 &v) {
    float invLen = 1.0f / std::sqrt(dot(v, v));
    return make_float3(v.x * invLen, v.y * invLen, v.z * invLen);
}

inline  float3 cross(const float3 &a, const float3 &b) {
    return make_float3(
        a.y*b.z - a.z*b.y,
        a.z*b.x - a.x*b.z,
        a.x*b.y - a.y*b.x
    );
}

struct Ray
{
    float3 origin;
    float3 direction;

    Ray() {}

    Ray(float3 o, float3 d) : origin(o), direction(d) {}

    float3 at(float t) const {return origin + t*direction;}

};

template<typename T>
inline constexpr const T& clamp(const T& v, const T& lo, const T& hi) {
    return (v < lo) ? lo : (hi < v) ? hi : v;
}

inline float3 clampf3(float3 v, float minVal, float maxVal) {
    return make_float3(
        clamp(v.x, minVal, maxVal),
        clamp(v.y, minVal, maxVal),
        clamp(v.z, minVal, maxVal)
    );
}

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
    int success = stbi_write_png("test_image.png", w, h, 3, imageData.data(), stride_in_bytes);

    if (!success) {
        std::cout << "Failed to write image.\n";
    }
}

struct Triangle
{
    int aInd;
    int bInd;
    int cInd;
    int naInd, nbInd, ncInd; // Normal indices

    float3 emission;
    float3 color;

    Triangle() {}

    Triangle(int a, int b, int c, int na, int nb, int nc, float3 col, float3 e)
        : aInd(a), bInd(b), cInd(c), naInd(na), nbInd(nb), ncInd(nc), color(col), emission(e) {}
};


inline void readObjSimple(
    std::string filename, 
    std::vector<float3>& points, 
    std::vector<float3>& normals,
    std::vector<Triangle>& mesh, 
    float3 c, float3 e
)
{
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open OBJ file with path " << filename << std::endl;
        return;
    }
    int startIndex = points.size();
    int normalStartIndex = normals.size();

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#' || line[0] == 's') continue; // skip comments

        std::istringstream iss(line);
        std::string prefix;
        
        iss >> prefix;
        

        if (prefix == "v") {
            double x, y, z;
            iss >> x >> y >> z;
            float3 p = make_float3(x, y, z);
            points.push_back(p);
        }
        else if (prefix == "vt") 
        {
        
        }
        else if (prefix == "vn") {
            double x, y, z;
            iss >> x >> y >> z;

            if (iss.fail() || std::isnan(x) || std::isnan(y) || std::isnan(z)) {
                normals.push_back(make_float3(0.0f, 1.0f, 0.0f)); // Safe dummy default
                continue;
            }
            float3 n = make_float3((float)x, (float)y, (float)z);
    
            float lenSq = lengthSquared(n);
            if (lenSq < 1e-12f) {
                n = make_float3(0.0f, 1.0f, 0.0f);
            }
            normals.push_back(n);
        }
        else if (prefix == "f") {
            std::vector<std::string> items;

            std::string vertinfo;
            std::vector<int> vertexIndices;
            std::vector<int> normalIndices;
            std::vector<int> uvIndices;
            while (iss >> vertinfo) 
            {
                std::istringstream vss(vertinfo);
                std::string idx;

                if (getline(vss, idx, '/'))
                {
                    if (!idx.empty())
                        vertexIndices.push_back(stoi(idx) - 1);
                }
                if (getline(vss, idx, '/'))
                {
                    if (!idx.empty())
                        uvIndices.push_back(stoi(idx) - 1);
                }
                if (getline(vss, idx, '/'))
                {
                    if (!idx.empty())
                        normalIndices.push_back(stoi(idx) - 1);
                }
            }
            bool hasUV = uvIndices.size() == vertexIndices.size();
            bool hasN  = normalIndices.size() == vertexIndices.size();
            int n = vertexIndices.size();
            // Triangulate the polygon as a fan from the first vertex
            for (int i = 1; i < n - 1; ++i) {
                bool isLight = lengthSquared(e) > 0;

                int idx0 = vertexIndices[0] + startIndex;
                int idx1 = vertexIndices[i] + startIndex;
                int idx2 = vertexIndices[i + 1] + startIndex;

                float3 p0 = points[idx0];
                float3 p1 = points[idx1];
                float3 p2 = points[idx2];

                float3 e1 = f3(p1.x - p0.x, p1.y - p0.y, p1.z - p0.z);
                float3 e2 = f3(p2.x - p0.x, p2.y - p0.y, p2.z - p0.z);
                
                float3 cp = cross(e1, e2);
                float area = 0.5f * length(cp);

                if (area < 1e-18f) {
                    continue; 
                }

                int n_idx0  = hasN ? normalIndices[0] + normalStartIndex : -1;
                int n_idx1  = hasN ? normalIndices[i] + normalStartIndex : -1;
                int n_idx2  = hasN ? normalIndices[i + 1] + normalStartIndex : -1;

                Triangle tri;
                if (isLight)
                    tri = Triangle(idx0, idx1, idx2, n_idx0, n_idx1, n_idx2, c, e);
                else
                    tri = Triangle(idx0, idx1, idx2, n_idx0, n_idx1, n_idx2, c, f3());
                mesh.push_back(tri);
            }
        }
    }

    file.close();
}

inline float randomFloat() {
    static std::mt19937 generator(std::random_device{}());
    static std::uniform_real_distribution<float> distribution(0.0f, 1.0f);
    
    return distribution(generator);
}