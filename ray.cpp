#include <iostream>
#include <cmath>

#include "vector.h"
#include "colour.h"
#include "material.h"
#include "object.h"
#include "intersection.h"
#include "ray.h"
#include "light.h"


Ray::Ray() {}

Ray::Ray(Vector r_origin, Vector r_D, float r_current_ri) {
    origin = r_origin;
    D = r_D.normalise();
    current_ri = r_current_ri;
}

Intersection Ray::nearestIntersect(Render::Object* objects[], int N_OBJECTS, int suppress_id) {

    double nearest_distance = 1e10;         // hard-coded max distance
    Intersection nearest_intersection = Intersection();

    for (int i = 0; i < N_OBJECTS; ++i) {
        Render::Object* object = objects[i];
        
        if (object->id != suppress_id) {
            Intersection is = object->intersection(this);
            if ((is.intersects == true) && (is.distance < nearest_distance)) {
                nearest_intersection = is;
                nearest_distance = is.distance;
            }
        }
    }

    return nearest_intersection;
}

Colour Ray::objectColour(
    Render::Object* objects[], 
    int N_OBJECTS,
    Colour background_colour) {

    Intersection terminal_intersection = nearestIntersect(objects, N_OBJECTS);

    if (terminal_intersection.intersects == false) {
        return background_colour;
    }

    return terminal_intersection.object_ptr->colour;
}

Colour Ray::shadedColour(
    Render::Object* objects[], 
    int N_OBJECTS,
    Colour background_colour) {

    Intersection terminal_intersection = nearestIntersect(objects, N_OBJECTS);

    if (terminal_intersection.intersects == false) {
        return background_colour;
    }

    float angle = terminal_intersection.normal.angleBetween(Vector(0, 1, 0));
    float scale = 1 - (terminal_intersection.normal.angleBetween(Vector(0, 1, 0)) / 3.14159);

    return terminal_intersection.object_ptr->colour.scaleRGB(scale);
}


Colour Ray::aggColour(
    Render::Object* objects[], 
    int N_OBJECTS,
    Colour background_colour,
    GlobalLight* global_lights,
    int N_GLOBAL_LIGHTS,
    PointLight* point_lights,
    int N_POINT_LIGHTS,
    int bounces,
    int max_bounces, 
    int space_ri, 
    int suppress_id) {

    Intersection terminal_intersection = Intersection();

    // max bounces 
    if (bounces > max_bounces) {
        terminal_intersection.normal = D;
        Colour col = terminal_intersection.directionRGB(background_colour, global_lights, N_GLOBAL_LIGHTS);
        return col;
    }

    // nearest intersection
    terminal_intersection = nearestIntersect(objects, N_OBJECTS, suppress_id);

    // nothing hit
    if (terminal_intersection.intersects == false) {
        terminal_intersection.normal = D;
        Colour col = terminal_intersection.directionRGB(background_colour, global_lights, N_GLOBAL_LIGHTS);
        return col;
    }

    // REFLECTION
    if (terminal_intersection.object_ptr->material_ptr->reflective == true) {
    
        origin = terminal_intersection.point;
        D = D.reflectInVector(terminal_intersection.normal);
        bounces += 1;
        
        if (terminal_intersection.object_ptr->material_ptr->reflection_ratio == 1) {
            return aggColour(objects, N_OBJECTS, background_colour, global_lights, N_GLOBAL_LIGHTS, point_lights, N_POINT_LIGHTS, bounces, max_bounces, space_ri);
        }

        Colour col = terminal_intersection.terminalRGB(background_colour, objects, N_OBJECTS, global_lights, N_GLOBAL_LIGHTS, point_lights, N_POINT_LIGHTS, 0);

        return aggColour(objects, N_OBJECTS, background_colour, global_lights, N_GLOBAL_LIGHTS, point_lights, N_POINT_LIGHTS, bounces, max_bounces, space_ri).avg(
            col, terminal_intersection.object_ptr->material_ptr->reflection_ratio, 1-terminal_intersection.object_ptr->material_ptr->reflection_ratio
        );

    }

    // REFRACTION
    if (terminal_intersection.object_ptr->material_ptr->transparent == true) {

        // Get weighted average between reflection and refraction
        
        Vector refracted_ray_D;
        float new_ri;

        Vector reflected_ray_D;

        // entering an object
        if (terminal_intersection.is_entering == true) {
            refracted_ray_D = D.refractInVector(terminal_intersection.normal, current_ri, terminal_intersection.object_ptr->material_ptr->refractive_index);
            new_ri = terminal_intersection.object_ptr->material_ptr->refractive_index;

            reflected_ray_D = D.reflectInVector(terminal_intersection.normal);
        }

        // leaving an object
        else {
            Render::Object* other_object = containedBy(terminal_intersection.point, objects, N_OBJECTS, terminal_intersection.object_ptr->id);

            // leaving into space
            if (other_object == NULL) {
                refracted_ray_D = D.refractInVector(terminal_intersection.normal, space_ri, terminal_intersection.object_ptr->material_ptr->refractive_index);
                new_ri = space_ri;

                reflected_ray_D = D.reflectInVector(terminal_intersection.normal);
            }

            // leaving into another object
            else {
                float other_ri = other_object->material_ptr->refractive_index;
                refracted_ray_D = D.refractInVector(terminal_intersection.normal, other_ri, terminal_intersection.object_ptr->material_ptr->refractive_index);
                new_ri = other_ri;

                reflected_ray_D = D.reflectInVector(terminal_intersection.normal);
            }
        }

        // TIR
        // no loss
        if (refracted_ray_D.valid == false) {
            refracted_ray_D = D.reflectInVector(terminal_intersection.normal);
            D = refracted_ray_D;

            origin = terminal_intersection.point;
            bounces += 1;

            return aggColour(objects, N_OBJECTS, background_colour, global_lights, N_GLOBAL_LIGHTS, point_lights, N_POINT_LIGHTS, bounces, max_bounces, space_ri);
        }

        // NORMAL REFRACTION
        // accounts for loss to reflection

        origin = terminal_intersection.point;
        bounces += 1;

        // refraction ray
        Ray refraction_ray = Ray(origin, refracted_ray_D, new_ri);
        Colour refraction_colour = refraction_ray.aggColour(objects, N_OBJECTS, background_colour, global_lights, N_GLOBAL_LIGHTS, point_lights, N_POINT_LIGHTS, bounces, max_bounces, space_ri);

        if (terminal_intersection.is_entering == false) {
            return refraction_colour;
        }

        // reflection ray
        Ray reflection_ray = Ray(origin, reflected_ray_D, current_ri);
        Colour reflection_colour = reflection_ray.aggColour(objects, N_OBJECTS, background_colour, global_lights, N_GLOBAL_LIGHTS, point_lights, N_POINT_LIGHTS, bounces, max_bounces, space_ri);

        float incident_angle = D.angleBetween(terminal_intersection.normal.invert());
        float r_factor = incident_angle*incident_angle * terminal_intersection.object_ptr->material_ptr->reflection_ratio;

        return refraction_colour.avg(reflection_colour, 2.5-r_factor, r_factor);
    }

    // NORMAL OBJECT TERMINUS
    Colour col = terminal_intersection.terminalRGB(background_colour, objects, N_OBJECTS, global_lights, N_GLOBAL_LIGHTS, point_lights, N_POINT_LIGHTS, 0);
    return col;
}


Render::Object* Ray::containedBy(Vector point, Render::Object* objects[], int N_OBJECTS, int suppress_id) {

    double min_contained_dist = 1e10;
    Render::Object* contained_by = NULL;

    for (int i = 0; i < N_OBJECTS; ++i) {

        // id not in suppress_ids
        if (objects[i]->id != suppress_id) {
            double is_contained_by = objects[i]->isInside(point);
            if (is_contained_by > 0) {
                if (is_contained_by < min_contained_dist) {
                    contained_by = objects[i];
                    min_contained_dist = is_contained_by;
                }
            }
        }
    }

    return contained_by;
}