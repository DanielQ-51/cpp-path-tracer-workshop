# ESAP Path Tracing Workshop
## By Daniel

This is a workshop for implementing a very basic naive path tracer. Feel free to look around the different files, but your task is just to fill in the function renderPixel() in render.cpp. You will do so by managing the core path tracing logic and math, while using provided functions to help you.

You should use traceClosestHit() in intersections.h, fetchGeometry() in intersections.h, and sampleBSDF() in bsdf.h. You can look them up in their respective files for their documentation.

Good luck and have fun!

Use the following command to compile:

```bash
g++ -O3 -std=c++17 -fopenmp main.cpp render.cpp -o render.exe
```

And use the following command to run:

```bash
./render.exe
```
