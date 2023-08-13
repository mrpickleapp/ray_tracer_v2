#include <iostream>
#include <cmath>

#include "vector.h"
#include "colour.h"
#include "material.h"
#include "object.h"
#include "intersection.h"
#include "ray.h"
#include "light.h"



Intersection::Intersection() {
    intersects = false;
}

Intersection::Intersection(bool i_intersects, bool i_is_entering, double i_distance, Vector i_point, Vector i_normal, Render::Object* i_object_ptr) {
    intersects = i_intersects;
    is_entering = i_is_entering;
    distance = i_distance;
    point = i_point;
    normal = i_normal;
    object_ptr = i_object_ptr;
}

Colour Intersection::directionRGB(Colour background_colour, GlobalLight* global_light_sources, int N_GLOBAL) {

    Colour global_lighting = background_colour;

    for (int i = 0; i < N_GLOBAL; ++i) {
        GlobalLight light = global_light_sources[i];
        float angle_to_light = normal.angleBetween(light.vector);
        global_lighting = global_lighting.addColour(light.relativeStrength(angle_to_light, distance));
    }

    return global_lighting.ceil();
}

Colour Intersection::terminalRGB(Colour background_colour, Render::Object* objects[], int N_OBJECTS, GlobalLight* global_light_sources, int N_GLOBAL, PointLight* point_light_sources, int N_POINT, int max_bounces) {

    Colour illumination = background_colour;

    if (object_ptr->material_ptr->emitive == true) {
        illumination = object_ptr->colour;
    }

    for (int i = 0; i < N_GLOBAL; ++i) {
        GlobalLight light = global_light_sources[i];
        double angle_to_light = normal.angleBetween(light.vector);
        illumination = illumination.addColour(light.relativeStrength(angle_to_light, 0));
    }

    for (int i = 0; i < N_POINT; ++i) {
        PointLight light = point_light_sources[i];
        if (object_ptr->id != light.id) {
            Vector vector_to_light = light.position.subtractVector(point);
            Ray ray_to_light = Ray(point, vector_to_light);

            // used to suppress the self object
            Intersection is = ray_to_light.nearestIntersect(objects, N_OBJECTS);    // assumes no bounces - straight line only

            if (is.intersects == true) {            // some rays not hitting target?
                if (is.object_ptr->id == light.id ) {
                    double angle_to_light = normal.angleBetween(vector_to_light);
                    double distance_to_light = vector_to_light.magnitude();
                    illumination = illumination.addColour(light.relativeStrength(angle_to_light, distance_to_light));
                }
            }
        }
    }

    return object_ptr->colour.illuminate(illumination).ceil();
}