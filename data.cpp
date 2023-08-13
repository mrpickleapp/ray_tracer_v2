#include "data.h"

#include "vector.h"
#include "colour.h"
#include "material.h"
#include "object.h"
#include "intersection.h"
#include "ray.h"
#include "light.h"
#include "physics.h"
#include "collision.h"

#include <windows.h>

#include <limits>

#define INF_MASS 10e25

Lib::Lib lib = Lib::Lib();

Sim::Data::Data(Scene::Data* sim) {

    /*
    
    Phys2D::Circle *circle_1 = new Phys2D::Circle(N_PHYS2D_OBJECTS, true, Vector(1, 1.2, 0), 0.4, &lib.reflective_glass, lib.grey);
    phys2D_objects[N_PHYS2D_OBJECTS++] = circle_1;
    circle_1->velocity = Vector(-2, 0, 0);

    Phys2D::Circle *circle_2 = new Phys2D::Circle(N_PHYS2D_OBJECTS, true, Vector(1.8, 1, 0), 0.25, &lib.mirror, lib.grey);
    phys2D_objects[N_PHYS2D_OBJECTS++] = circle_2;

    Phys2D::Circle *circle_3 = new Phys2D::Circle(N_PHYS2D_OBJECTS, true, Vector(-2, 0.5, 0), 0.5, &lib.water, lib.grey);
    phys2D_objects[N_PHYS2D_OBJECTS++] = circle_3;
    
    Phys2D::Circle *circle_4 = new Phys2D::Circle(N_PHYS2D_OBJECTS, true, Vector(-1.75, 1.5, 0), 0.15, &lib.mirror, lib.grey);
    phys2D_objects[N_PHYS2D_OBJECTS++] = circle_4;

    */

    /*

    Vector* poly_2_vectors = new Vector[3] {
        Vector(1, 0, 0),
        Vector(2, 0.2, 0),
        Vector(0.8, 0.15, 0),
    };
    Phys2D::Polygon *poly_2 = new Phys2D::Polygon(N_PHYS2D_OBJECTS, true, Vector(1, 0.1, 0), 5, poly_2_vectors, 3, &lib.emitive, Colour(200, 220, 240));
    poly_2->is_fixed = false;
    poly_2->has_gravity = false;
    poly_2->mass = INF_MASS;
    poly_2->moment_of_inertia = 0.01;
    poly_2->angular_velocity_Z = 3;
    // poly_2->velocity = Vector(4, 0, 0);

    phys2D_objects[N_PHYS2D_OBJECTS++] = poly_2;
    */

    int gear_teeth = 37;
    double gear_radius = 0.6;

    int n_esc_points = 8;
    Vector* esc_points = new Vector[n_esc_points] {
        Vector(0, 1.4, 0),

        Vector(-0.8, 1, 0),
        Vector(-0.45, 0.85, 0),
        Vector(-0.6, 1, 0),

        Vector(0, 1.1, 0),

        Vector(0.6, 1, 0),
        Vector(0.5, 0.75, 0),
        Vector(0.8, 1, 0)
    };

    Phys2D::Polygon *esc_1 = new Phys2D::Polygon(N_PHYS2D_OBJECTS, true, Vector(0, 0, 0), 1, esc_points, n_esc_points, &lib.dull_mirror, Colour(181, 166, 66), 0.2);
    esc_1->has_gravity = false;
    esc_1->mass = INF_MASS;
    esc_1->moment_of_inertia = 1;
    esc_1->has_pivot = true;
    esc_1->pivot = Vector(0, 1.3, 0);
    esc_1->centre_of_mass = Vector(0, 0, 0);
    esc_1->angular_velocity_Z = 0.35;
    phys2D_objects[N_PHYS2D_OBJECTS++] = esc_1;

    Phys2D::Polygon *gear_2 = Phys2D::MakeGear(N_PHYS2D_OBJECTS, true, Vector(0.7, 0, 0), Vector(0.3, 0.2, 0), gear_teeth, gear_radius, &lib.emitive, lib.grey);
    gear_2->is_fixed = false;
    gear_2->has_gravity = false;
    gear_2->mass = INF_MASS;
    gear_2->moment_of_inertia = 0.1;
    gear_2->angular_velocity_Z = 0;
    // phys2D_objects[N_PHYS2D_OBJECTS++] = gear_2;

    /*

    Phys2D::Line *line = new Phys2D::Line(N_PHYS2D_OBJECTS, true, Vector(1.8, 1, 0), Vector(0.8, 0, 0),  0.05, &lib.base_material, lib.grey);
    line->mass = 1;
    line->CalcMoI();
    line->velocity = Vector(-5, 0, 0);
    line->angular_velocity_Z = 3;
    phys2D_objects[N_PHYS2D_OBJECTS++] = line;

    Phys2D::Line *line_10 = new Phys2D::Line(N_PHYS2D_OBJECTS, true, Vector(-1.8, 0, 0), Vector(-0.8, 1, 0), 0.05, &lib.base_material, lib.grey);
    line_10->mass = 1;
    line_10->CalcMoI();
    phys2D_objects[N_PHYS2D_OBJECTS++] = line_10;
    */

    /*

    Phys2D::Line *line_1 = new Phys2D::Line(N_PHYS2D_OBJECTS, true, Vector(-3.2, -1.5, 0), Vector(3.2, -1.5, 0), 0.2, &lib.reflective_glass, lib.grey);
    line_1->is_fixed = true;
    line_1->mass = INF_MASS;
    line_1->CalcMoI();
    phys2D_objects[N_PHYS2D_OBJECTS++] = line_1;

    Phys2D::Line *line_2 = new Phys2D::Line(N_PHYS2D_OBJECTS, true, Vector(-3.2, -1.5, 0), Vector(-3.5, 2, 0), 0.2, &lib.mirror, lib.grey);
    line_2->is_fixed = true;
    line_2->mass = INF_MASS;
    line_2->CalcMoI();
    phys2D_objects[N_PHYS2D_OBJECTS++] = line_2;

    Phys2D::Line *line_3 = new Phys2D::Line(N_PHYS2D_OBJECTS, true, Vector(3.2, -1.5, 0), Vector(3.5, 2, 0), 0.2, &lib.mirror, lib.grey);
    line_3->is_fixed = true;
    line_3->mass = INF_MASS;
    line_3->CalcMoI();
    phys2D_objects[N_PHYS2D_OBJECTS++] = line_3;

    Phys2D::Line *line_4 = new Phys2D::Line(N_PHYS2D_OBJECTS, true, Vector(-3.5, 2, 0), Vector(3.5, 2, 0), 0.2, &lib.reflective_glass, lib.grey);
    line_4->is_fixed = true;
    line_4->mass = INF_MASS;
    line_4->CalcMoI();
    phys2D_objects[N_PHYS2D_OBJECTS++] = line_4;

    */

    /*

    Vector* poly_1_vectors = new Vector[4] {
        Vector(0, 0, 0),
        Vector(0.5, 1, 0),
        Vector(0, 0.5, 0),
        Vector(-0.5, 1, 0)
    };
    Phys2D::Polygon *poly_1 = new Phys2D::Polygon(N_PHYS2D_OBJECTS, true, Vector(0, 0.5, 0), 5, poly_1_vectors, 4, &lib.emitive, Colour(200, 220, 240));
    poly_1->is_fixed = false;
    poly_1->has_gravity = false;
    poly_1->mass = INF_MASS;
    poly_1->moment_of_inertia = 0.01;
    poly_1->angular_velocity_Z = 0;

    phys2D_objects[N_PHYS2D_OBJECTS++] = poly_1;
    */


    /*
    for (int i = 0; i < 4; i++) {
        Phys2D::Circle *marker = new Phys2D::Circle(sim->GetFreeId(), true, poly_1_vectors[i], 0.025, &lib.base_material, Colour(0, 255, 0));
        marker->is_fixed = true;
        marker->can_collide = false;
        phys2D_objects[N_PHYS2D_OBJECTS++] = marker;
    }
    */

}

void Sim::Data::PhysicsIteration() {

    for (int iteration = 0; iteration < SIM_MULTIPLE; ++iteration) {

        // Bit grid to check which object<>object collisions have already been checked
        int check_grid_size = N_PHYS2D_OBJECTS * N_PHYS2D_OBJECTS;
        bool* check_grid = new bool[check_grid_size] {false}; 

        // way of checking which type of interaction is relevant, and sorting inputs


        // Check collisions
        for (int i = 0; i < N_PHYS2D_OBJECTS; ++i) {

            Phys2D::Object* phys_object = phys2D_objects[i];

            if (!phys_object->can_collide) {
                continue;
            }

            for (int j = 0; j < N_PHYS2D_OBJECTS; ++j) {

                // do not check object against itself
                if (i == j) {
                    continue;
                }

                // interaction has already been handled
                if (check_grid[i * N_PHYS2D_OBJECTS + j] | check_grid[j * N_PHYS2D_OBJECTS + i]) {
                    continue;
                }

                // get second phys_object
                Phys2D::Object* phys_object_2 = phys2D_objects[j];

                if (!phys_object_2->can_collide) {
                    continue;
                }

                // generalised collision handler

                Phys2D::Collision collision = Phys2D::Collision(phys_object, phys_object_2);

                collision.ClosestPoints();

                if (collision.objects_collide) {
                    
                    collision.PointsConverging();

                    if (collision.objects_converging) {

                        collision.Resolve(this);
                    }
                }

                /*

                std::cout << "START" << std::endl;

                std::cout << collision.distance << std::endl;
                collision.normal.describe();
                collision.point_a.point.describe();
                collision.point_b.point.describe();
                collision.relative_point_velocity.describe();
                std::cout << collision.objects_collide << std::endl;
                std::cout << collision.objects_converging << std::endl;

                std::cout << "END" << std::endl << std::endl;

                */


                // set check grid
                check_grid[i * N_PHYS2D_OBJECTS + j] = true;
                check_grid[j * N_PHYS2D_OBJECTS + i] = true;
            }
        }

        delete [] check_grid;


        // Apply forces
        for (int i = 0; i < N_PHYS2D_OBJECTS; ++i) {

            phys2D_objects[i]->IteratePhysics(this);
        }
    }
}

void Scene::Data::updateFromPhysObjects(Sim::Data sim) {

    for (int i = 0; i < sim.N_PHYS2D_OBJECTS; ++i) {

        int o = i + N_BASE_OBJECTS;

        delete objects[o];

        objects[o] = sim.phys2D_objects[i]->MakeRenderAlias();
    }

    N_OBJECTS = N_BASE_OBJECTS + sim.N_PHYS2D_OBJECTS;
}

Scene::Data::Data() {
    // Testing
    GlobalLight *gl1 = new GlobalLight(0, Vector(0.5, 1, 0), Colour(25, 75, 255), 1, lib.ninety_degrees, 0);
    global_lights[N_GLOBAL_LIGHTS++] = *gl1;

    GlobalLight *gl2 = new GlobalLight(0, Vector(-0.5, 1, 0), Colour(255, 75, 25), 1, lib.ninety_degrees, 0);
    global_lights[N_GLOBAL_LIGHTS++] = *gl2;


    int s = 3;
    Vector base_vector = Vector(0, 3, -3);

    for (int i = 0; i < s; i++) {
        PointLight *pl = new PointLight(GetFreeId(), base_vector.rotateExtrinsic(0, (6.28319 / s) * i, 0), Colour(250, 233, 192), 1, lib.ninety_degrees, -1);
        point_lights[i] = *pl;
        N_POINT_LIGHTS++;
    }

    for (int i=0; i<N_POINT_LIGHTS; ++i) {
        PointLight pl = point_lights[i];
        AddSphere(pl.id, pl.position, 0.5, &lib.emitive, pl.colour);
    }

    Render::Sphere *globe = new Render::Sphere(GetFreeId(), Vector(0, 2, 0), 0.5, &lib.reflective_glass, lib.grey);
    objects[N_OBJECTS++] = globe;

    /*

    Render::Cylinder *cyl_1 = new Render::Cylinder(GetFreeId(), Vector(-1, -1, 1), Vector(-1.5, -1.5, 3), 0.5, &lib.dull_mirror, Colour(181, 166, 66));
    objects[N_OBJECTS++] = cyl_1;

    */

    /*
    Vector *ply_1_vertices = new Vector[5] {
        Vector(-0.5, 0.25, 0), 
        Vector(0, 1, 0), 
        Vector(-1, 0.9, 0),
        Vector(-1.2, -0.5, 0),
        Vector(-0.1, -0.2, 0)
    };
    
    Render::Polygon *ply_1 = new Render::Polygon(1, ply_1_vertices, 5, &lib.base_material, lib.grey);
    objects[2] = ply_1;
    */

    /*
    Vector *pm_vertices = new Vector[4] {
        Vector(-2, 2, -1),
        Vector(-3, 1, 0),
        Vector(-1, 1, 0),
        Vector(-2, 1, -2)
    };
    Render::Pyramid *pm_1 = new Render::Pyramid(3, pm_vertices, &lib.base_material, Colour(225, 180, 110));
    objects[4] = pm_1;
    */

    /*
    Render::FacetedDiamond *diamond = new Render::FacetedDiamond(
        GetFreeId(), 
        Vector(0, 1, 0), 
        Vector(0, -1.5, 0), 
        15, 
        &lib.diamond, 
        lib.grey
    );
    objects[N_OBJECTS++] = diamond;

    Render::Cylinder *floor = new Render::Cylinder(GetFreeId(), Vector(0, -1.8, 0), Vector(0, -2, 0), 3, &lib.mirror, lib.grey);
    objects[N_OBJECTS++] = floor;

    Render::Cylinder *floor_2 = new Render::Cylinder(GetFreeId(), Vector(0, -2.5, 0), Vector(0, -2, 0), 8, &lib.base_material, Colour(70, 90, 110));
    objects[N_OBJECTS++] = floor_2;

    Render::Pipe *ring = new Render::Pipe(GetFreeId(), Vector(0, 5, 0), Vector(0, -1, 0), 10, &lib.emitive, Colour(250, 233, 192));
    objects[N_OBJECTS++] = ring;
    */


    N_BASE_OBJECTS = N_OBJECTS;
    
    /*

    PointLight *pl1 = new PointLight(100, Vector(3, 8, 5), Colour(250, 233, 192), 1, lib.ninety_degrees, -1);
    PointLight *pl2 = new PointLight(101, Vector(-3, 8, -5), Colour(250, 233, 192), 1, lib.ninety_degrees, -1);
    PointLight *pl3 = new PointLight(100, Vector(3, 8, -5), Colour(250, 233, 192), 1, lib.ninety_degrees, -1);
    PointLight *pl4 = new PointLight(101, Vector(-3, 8, 5), Colour(250, 233, 192), 1, lib.ninety_degrees, -1);
    point_lights[0] = *pl1;
    point_lights[1] = *pl2;
    point_lights[2] = *pl3;
    point_lights[3] = *pl4;
    N_POINT_LIGHTS = 4;

    for (int i=0; i<N_POINT_LIGHTS; ++i) {
        PointLight pl = point_lights[i];
        AddSphere(pl.id, pl.position, 0.5, &lib.emitive, pl.colour);
    }

    Render::Sphere *ball_1 = new Render::Sphere(20, Vector(-0.5, 0.5, 2), 0.6, &lib.water, lib.grey);
    objects[4] = ball_1 ;
    Render::Cylinder *floor = new Render::Cylinder(22, Vector(0, -1.8, 0), Vector(0, -2, 0), 2, &lib.mirror, lib.grey);
    objects[5] = floor;
    Render::Cylinder *cyl_1 = new Render::Cylinder(21, Vector(1, -0.8, 0), Vector(1, -1.5, 0), 1.2, &lib.water, lib.grey);
    objects[6] = cyl_1;
    Render::Cylinder *straw = new Render::Cylinder(23, Vector(1.5, -0.2, -0.2), Vector(0.9, -1.7, 0.2), 0.1, &lib.base_material, lib.grey);
    objects[7] = straw;
    Render::Sphere *bubble_1 = new Render::Sphere(24, Vector(0.8, -1.1, 0.2), 0.075, &lib.air, lib.grey);
    objects[8] = bubble_1;
    Render::Cylinder *cyl_2 = new Render::Cylinder(25, Vector(-0.5, 1.2, -1), Vector(1.9, 1.8, -0.8), 0.5, &lib.glass, lib.grey);
    objects[9] = cyl_2;
    Render::Sphere *bubble_2 = new Render::Sphere(26, Vector(0.1, 1, -0.9), 0.075, &lib.air, lib.grey);
    objects[10] = bubble_2;
    Render::Sphere *bubble_3 = new Render::Sphere(27, Vector(-0.6, 0.8, 2.1), 0.075, &lib.air, lib.grey);
    objects[11] = bubble_3;
    Render::Sphere *ball_2 = new Render::Sphere(28, Vector(1.5, 0.5, -3), 1, &lib.mirror, lib.grey);
    objects[12] = ball_2;
    
    N_OBJECTS = 11;

    */


}

Scene::Data::Data(int _width, int _height) {
    SetDims(_width, _height);
}

void Scene::Data::SetDims(int _width, int _height) {
    X_DIM = _width + _width%2;
    Y_DIM = _height + _height%2;
    PIXEL_BUFFER = VirtualAlloc(0, _width * _height * sizeof(uint32_t), MEM_RESERVE|MEM_COMMIT, PAGE_READWRITE);
    TOTAL_RAYS = X_DIM * Y_DIM;
    InitRays();
}

void Scene::Data::InitRays() {
    RAY_STEP = 0.005 * (200.0 / X_DIM) * FOV; 
    // GENERATE OFFSET GRID
    X_RAYS = new float[X_DIM];
    for (int i = 0; i < X_DIM; ++i) {
        int n = -X_DIM/2 + i;
        X_RAYS[i] = RAY_STEP * n;
    }

    Y_RAYS = new float[Y_DIM];
    for (int i = 0; i < Y_DIM; ++i) {
        int n = Y_DIM/2 - i;
        Y_RAYS[i] = RAY_STEP * n;
    }
}

int Scene::Data::AddSphere(double _x, double _y, double _z, float _radius) {

    int id = GetFreeId();
    
    Render::Sphere *sphere = new Render::Sphere(id, Vector(_x, _y, _z), _radius, lib.RandomMaterial(), lib.grey);
    objects[N_OBJECTS] = sphere;
    N_OBJECTS ++;

    return id;
}

int Scene::Data::AddSphere(int _id, Vector _pos, float _radius, Material* _material, Colour _colour) {
    
    Render::Sphere *sphere = new Render::Sphere(_id, _pos, _radius, _material, _colour);
    objects[N_OBJECTS] = sphere;
    N_OBJECTS ++;

    return _id;
}

int Scene::Data::AddSphere(int _x, int _y, Vector _v) {
    
    float x = X_RAYS[_x];
    float y = Y_RAYS[_y];
    
    Vector X_RAY_STEP = Vector(1, 0, 0).rotateIntrinsic(CAMERA_ROT_Z, CAMERA_ROT_Y, CAMERA_ROT_X);
    Vector Y_RAY_STEP = Vector(0, 1, 0).rotateIntrinsic(CAMERA_ROT_Z, CAMERA_ROT_Y, CAMERA_ROT_X);

    Vector ray_origin = camera_position;
    Vector ray_D = camera_direction.addVector(X_RAY_STEP.scaleByLength(x)).addVector(Y_RAY_STEP.scaleByLength(y)).normalise();
    Ray ray = Ray(ray_origin, ray_D);
    
    double denom = ray.D.dotProduct(Vector(0, 0, -1));
    Vector p = Vector(0, 0, 0).subtractVector(ray.origin);
    double t = p.dotProduct(Vector(0, 0, -1)) / denom;
    p = ray.origin.addVector(ray.D.scaleByLength(t));

    return AddSphere(p.x, p.y, 0, 0.25);
}

int Scene::Data::GetFreeId() {

    int free_id = 0;

    bool ok = true;

    do {
        ok = true;
        for (int i = 0; i < N_OBJECTS; ++i) {
            if (free_id == objects[i]->id) {
                ++free_id;
                ok = false;
            }
        }
        for (int i = 0; i < N_GLOBAL_LIGHTS; ++i) {
            if (free_id == global_lights[i].id) {
                ++free_id;
                ok = false;
            }
        }
        for (int i = 0; i < N_POINT_LIGHTS; ++i) {
            if (free_id == point_lights[i].id) {
                ++free_id;
                ok = false;
            }
        }
    }
    while (ok == false);
    
    return free_id;
}

