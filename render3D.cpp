#include "render3D.h"

#include <iostream>
#include <iomanip>
#include <string>
#include <cstdlib>
#include <chrono>
#include <sstream>

#include "vector.h"
#include "colour.h"
#include "material.h"
#include "object.h"
#include "intersection.h"
#include "ray.h"
#include "light.h"

#define STBI_MSC_SECURE_CRT
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"


int render3D::render(
    const char* filename,
    Object* objects[],
    int N_OBJECTS,
    PointLight point_lights[],
    int N_POINT_LIGHTS,
    GlobalLight global_lights[],
    int N_GLOBAL_LIGHTS,
    int X_DIM,
    int Y_DIM,
    float FOV,
    float CAMERA_DIST,
    float CAMERA_ROT_Y,
    float SPACE_RI,
    int MAX_BOUNCES
) {

    const float RAY_STEP = 0.005 * (200.0 / X_DIM) * FOV;   
    const int TOTAL_RAYS = X_DIM * Y_DIM;
    
    const float Z = -1;

    // GENERATE OFFSET GRID
    float* X_RAYS = new float[X_DIM];
    for (int i = 0; i < X_DIM; ++i) {
        int n = -X_DIM/2 + i;
        X_RAYS[i] = RAY_STEP * n;
    }

    float* Y_RAYS = new float[Y_DIM];
    for (int i = 0; i < Y_DIM; ++i) {
        int n = Y_DIM/2 - i;
        Y_RAYS[i] = RAY_STEP * n;
    }

    Vector ray_origin = Vector(0, 0, CAMERA_DIST).rotateY(CAMERA_ROT_Y);
    Ray ray = Ray(ray_origin, Vector(0, 0, -1), SPACE_RI);
    Colour background_colour = Colour(0, 0, 0);

    uint8_t* pixels = new uint8_t[TOTAL_RAYS * 3];

    // RAY-TRACING LOOP
    for (int Y = 0; Y < Y_DIM; ++Y) {
        for (int X = 0; X < X_DIM; ++X) {

            float x = X_RAYS[X];
            float y = Y_RAYS[Y];
            
            // have to update ray!
            // would be quicker to calculate x, y, z offset and then just to additions
            ray.origin = ray_origin;
            ray.D = Vector(x, y, Z).normalise().rotateY(CAMERA_ROT_Y);
            ray.current_ri = SPACE_RI;

            // NEW RAY GRID
            for (int i = 0; i < X_DIM; ++i) {
                int n = -X_DIM/2 + i;
                X_RAYS[i] = RAY_STEP * n;
            }

            int i1 = (Y*X_DIM + X) * 3;

            Colour col = ray.aggColour(objects, N_OBJECTS, background_colour, global_lights, N_GLOBAL_LIGHTS, point_lights, N_POINT_LIGHTS, 0, MAX_BOUNCES);

            pixels[i1] = col.r;
            pixels[i1 + 1] = col.g;
            pixels[i1 + 2] = col.b;
        }
    }

    // SAVE PNG
    int comp = 3; // (RGB)
    int w = X_DIM;
    int h = Y_DIM;
    stbi_write_png_compression_level = 0;
    stbi_write_png(filename, w, h, comp, pixels, w*3);

    // CLEAR
    delete [] X_RAYS;
    delete [] Y_RAYS;

    // EXIT
    return *pixels;
}