#pragma once

#include "vector.h"
#include "colour.h"
#include "material.h"
#include "object.h"
#include "intersection.h"
#include "ray.h"
#include "light.h"
#include "physics.h"
#include "collision.h"

class Ray;
class Material;
class Colour;
class Intersection;
class Object;
class GlobalLight;
class PointLight;
namespace Phys2D {class Object;}
namespace Scene {class Data;}

const int MAX_OBJECTS = 100;
const int MAX_LIGHTS = 25;

namespace Sim {

    #define RENDER_COLOUR_MODE 0
    #define RENDER_SHADED_MODE 1
    #define RENDER_FULL_MODE 2

    #define PHYSICS_SIM_OFF 0
    #define PHYSICS_SIM_2D 1
    #define PHYSICS_SIM_3D 2

    #define MSAA_NORMAL 0
    #define MSAA_SUB_SAMPLE 1

    class Data {        // rename to handler
        public:
            bool MULTITHREADING = true;
            int SAMPLE_RATE = 1;
            int MSAA = 2;
            int MSAA_TYPE = MSAA_NORMAL;
            int MSAA_CONTRAST_THRESHOLD = 10;
            float TICK_MS = 10;                       // length of each render tick
            int SIM_MULTIPLE = 100;                 // sim iterations per tick
            int MODE = RENDER_FULL_MODE;
            bool RENDER_PAUSED = false;
            float CAMERA_ROT_INCR = 0.02;
            float CAMERA_TRANS_INC = 0.05;
            float FOV_INCR = 1.1;

            Render::Object* selected_object = NULL;

            bool PHYSICS_PAUSED = false;
            int PHYSICS_MODE = PHYSICS_SIM_2D;
            float GRAVITY = 9.81;
            float ELASTICITY = 0.1;

            Phys2D::Object* phys2D_objects[MAX_OBJECTS] {nullptr};
            int N_PHYS2D_OBJECTS = 0;

            Data(Scene::Data*);
            void PhysicsIteration();
    };
}

namespace Scene {

    class Data {        // rename to handler
        public:
            int X_DIM, Y_DIM;
            void* PIXEL_BUFFER;
            float RAY_STEP;
            int TOTAL_RAYS;
            int CAMERA_DIST = 2;
            float Z = -1;
            float CAMERA_ROT_Z = 0;
            float CAMERA_ROT_Y = 0;
            float CAMERA_ROT_X = 0;
            Vector camera_position = Vector(0.75, 0.5, CAMERA_DIST);
            Vector camera_direction = Vector(0, 0, Z);
            float SPACE_RI = 1;
            int MAX_BOUNCES = 10;

            float FOV = 1.4;

            float* X_RAYS;
            float* Y_RAYS;

            Colour background_colour = Colour(0, 0, 0);

            GlobalLight global_lights[MAX_LIGHTS];
            PointLight point_lights[MAX_LIGHTS];
            Render::Object* objects[MAX_OBJECTS] {nullptr};

            int N_GLOBAL_LIGHTS = 0;
            int N_POINT_LIGHTS = 0;
            int N_BASE_OBJECTS = 0;
            int N_OBJECTS = 0;

            Data();
            Data(int, int);
            void SetDims(int, int);
            void InitRays();
            int AddSphere(double=0, double=0, double=0, float=1);
            int AddSphere(int, Vector, float, Material*, Colour);
            int AddSphere(int, int, Vector);
            int GetFreeId();

            void updateFromPhysObjects(Sim::Data);
    };
}

namespace Lib {

    class Lib {
        public:
            Material base_material = Material();
            Material mirror = Material(true, false, false, 1, 1);
            Material dull_mirror = Material(true, false, false, 1, 0.25);
            Material shiny_mirror = Material(true, false, false, 1, 0.8);
            Material glass = Material(false, true, false, 1.51, 0.25);
            Material reflective_glass = Material(false, true, false, 1.51, 1);
            Material water = Material(false, true, false, 1.333, 0.5);
            Material air = Material(false, true, false, 1);
            Material emitive = Material(false, false, true, 1);
            Material diamond = Material(false, true, false, 2.417, 1);

            const Colour background_colour = Colour(0, 0, 0);
            const Colour grey = Colour(200, 200, 200);

            float ninety_degrees = 1.5708;

            Material* materials[4] {&base_material, &mirror, &glass, &water};
            int N_MATERIALS = 4;

            Material* RandomMaterial() {
                return materials[rand() % N_MATERIALS];
            }
    };
}

