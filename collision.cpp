#include <iostream>
#include <cmath>

#include "collision.h"

#include "vector.h"
#include "colour.h"
#include "material.h"
#include "object.h"
#include "intersection.h"
#include "ray.h"
#include "light.h"
#include "data.h"


Phys2D::Collision::Collision() {}

Phys2D::Collision::Collision(Phys2D::Object* _object_a, Phys2D::Object* _object_b) {
    object_a = _object_a;
    object_b = _object_b;
}

Phys2D::Point::Point() {}

void Phys2D::Collision::ClosestPoints() {

    // if no vertices on either (ie two circles), just check point a against point b
    if (object_a->n_sides == 0 & object_b->n_sides == 0) {
        Phys2D::Collision::PointPointDetection();
    }

    else if (object_a->n_sides > 0 & object_b->n_sides > 0) {

        int sides_intersect = Phys2D::Collision::LineLineDetection();

        if (sides_intersect == 1) {
            objects_collide = true;
            Phys2D::Collision::PointLineDetection(0);
            Phys2D::Collision::PointLineDetection(1);
        }
    }

}

Vector Phys2D::Point::PointVelocity() {

    point_angular_v = object_com_to_point.magnitude() * object->angular_velocity_Z;

    // rotate by 270 degrees and scale to angular velocity
    point_rotation_vector = object_com_to_point.crossProduct(Vector(0, 0, -1)).scaleToLength(point_angular_v);

    point_velocity = object->velocity.addVector(point_rotation_vector);

    return point_velocity;
}

Vector Phys2D::Collision::RelativePointVelocity() {

    relative_point_velocity = normal.scaleByLength(point_a.PointVelocity().subtractVector(point_b.PointVelocity()).dotProduct(normal));

    double point_a_v_on_normal = point_a.point_velocity.dotProduct(normal);
    double point_b_v_on_normal = point_b.point_velocity.dotProduct(normal);

    approach_factor = point_a_v_on_normal - point_b_v_on_normal;

    return relative_point_velocity;
}

bool Phys2D::Collision::PointsConverging() {

    RelativePointVelocity();
    objects_converging = approach_factor > 0 ? true : false;

    return objects_converging;
}

void Phys2D::Collision::CircleLineDetection() {

}

int Phys2D::Collision::LineLineDetection() {

    for (int i = 0; i < object_a->n_sides; ++i) {
        for (int j = 0; j < object_b->n_sides; ++j) {

            Vector is = object_a->sides[i].Intersect(object_b->sides[j]);

            if (is.valid) {
                return 1;
            }
        }
    }

    return 0;
}


void Phys2D::Collision::PointLineDetection(int direction) {

    Vector* points;
    int n_points;
    Phys::Side* sides;
    int n_sides;

    if (direction == 0) {
        points = object_a->coords;
        n_points = object_a->n_coords;
        sides = object_b->sides;
        n_sides = object_b->n_sides;
    }
    else {
        points = object_b->coords;
        n_points = object_b->n_coords;
        sides = object_a->sides;
        n_sides = object_a->n_sides;
    }

    for (int i = 0; i < n_points; ++i) {

        Vector point = points[i];

        for (int j = 0; j < n_sides; ++j) {

            Phys::Side side = sides[j];

            Vector line_a = point.subtractVector(side.end);
            Vector line_b = side.start.subtractVector(point);
            Vector line_c = side.vector.invert();

            double length_a = line_a.magnitude();
            double length_b = line_b.magnitude();
            double length_c = line_c.magnitude();

            // could prune here

            double angle_a = line_c.angleBetween(line_b);
            double angle_b = line_a.angleBetween(line_c);

            double collision_distance;
            Vector collision_point;
            Vector collision_normal;

            if (angle_a >= 1.5708) {
                
                collision_distance = length_b;
                collision_point = side.start;
                collision_normal = line_b;
            }

            else if (angle_b >= 1.5708) {

                collision_distance = length_a;
                collision_point = side.start;
                collision_normal = line_a.invert();
            }

            else {

                collision_point = side.start.subtractVector(line_c.scaleByLength(line_b.dotProduct(line_c) / length_c / length_c));
                collision_normal = collision_point.subtractVector(point);
                collision_distance = collision_normal.magnitude();
            }

            collision_distance -= object_a->radius + object_b->radius;

            if (collision_distance < distance) {

                collision_normal = collision_normal.normalise();

                distance = collision_distance;
                
                // set points
                // could only do this when we've found the nearest one for speed
                
                if (direction == 0) {
                    normal = collision_normal.invert();
                    point_a = Point();
                    point_a.object = object_a;
                    point_a.point = point.addVector(normal.invert().scaleToLength(object_a->radius));
                    point_a.object_com_to_point = point_a.point.subtractVector(object_a->pivot);

                    point_b = Point();
                    point_b.object = object_b;
                    point_b.point = collision_point.addVector(normal.scaleToLength(object_b->radius));
                    point_b.object_com_to_point = point_b.point.subtractVector(object_b->pivot);
                }

                else {
                    normal = collision_normal;
                    point_a = Point();
                    point_a.object = object_a;
                    point_a.point = collision_point.addVector(normal.invert().scaleToLength(object_a->radius));
                    point_a.object_com_to_point = point_a.point.subtractVector(object_a->pivot);

                    point_b = Point();
                    point_b.object = object_b;
                    point_b.point = point.addVector(normal.scaleToLength(object_b->radius));
                    point_b.object_com_to_point = point_b.point.subtractVector(object_b->pivot);
                }

                // output
                /*
                std::cout << "POINTS" << std::endl;
                std::cout << "Normal: "; normal.describe();
                std::cout << "Point A: ";point_a.point.describe();
                std::cout << "A com: "; point_a.object->centre_of_mass.describe();
                std::cout << "A com to point: ";point_a.object_com_to_point.describe();
                std::cout << "Point B: ";point_b.point.describe();
                std::cout << "B com: "; point_b.object->centre_of_mass.describe();
                std::cout << "B com to point: ";point_b.object_com_to_point.describe();
                std::cout << std::endl;
                */

            }
        }
    }
}


void Phys2D::Collision::Resolve(Sim::Data* sim) {

    /*
    std::cout << "COLLISION between id: " << object_a->id << " and id: " << object_b->id << std::endl;

    std::cout << "Point a: ";
    point_a.point.describe();
    std::cout << "Object a com: ";
    point_a.object->centre_of_mass.describe();
    std::cout << "Object com to point a: ";
    point_a.object_com_to_point.describe();
    std::cout << "Point a velocity: ";
    point_a.point_velocity.describe();

    std::cout << "Point b: ";
    point_b.point.describe();
    std::cout << "Object b com: ";
    point_b.object->centre_of_mass.describe();
    std::cout << "Object com to point b: ";
    point_b.object_com_to_point.describe();
    std::cout << "Point b velocity: ";
    point_b.point_velocity.describe();
    std::cout << "Normal: ";
    normal.describe();
    */

    Vector n = normal;
   
    double v_imp = n.dotProduct(relative_point_velocity);

    double mass_reduced = pow(
        1.0/object_a->mass + 1.0/object_b->mass + 
        n.crossProduct(point_a.object_com_to_point).dotProduct(
            n.crossProduct(point_a.object_com_to_point).scaleByLength(pow(object_a->moment_of_inertia, -1.0))
        ) + 
        n.crossProduct(point_b.object_com_to_point).dotProduct(
            n.crossProduct(point_b.object_com_to_point).scaleByLength(pow(object_b->moment_of_inertia, -1.0))
        ),
        -1.0
    );
    

    double impulse = -(1.0 + sim->ELASTICITY) * mass_reduced * v_imp;

    /*

    double k_energy_a = object_a->velocity.magnitude() * object_a->velocity.magnitude() * object_a->mass / 2.0;
    double gpe_energy_a = object_a->mass * (10.0 + object_a->centre_of_mass.y) * 9.81;
    double ang_energy_a = object_a->moment_of_inertia * object_a->angular_velocity_Z * object_a->angular_velocity_Z / 2.0;
    double energy_before_a = k_energy_a + gpe_energy_a + ang_energy_a;

    double k_energy_b = object_b->velocity.magnitude() * object_b->velocity.magnitude() * object_b->mass / 2.0;
    double gpe_energy_b = object_b->mass * (10.0 + object_b->centre_of_mass.y) * 9.81;
    double ang_energy_b = object_b->moment_of_inertia * object_b->angular_velocity_Z * object_b->angular_velocity_Z / 2.0;
    double energy_before_b = k_energy_b + gpe_energy_b + ang_energy_b;
    */

    if (object_a->is_fixed == false & object_b->is_fixed == false) {
        
        object_a->velocity = object_a->velocity.addVector(n.scaleByLength(impulse/object_a->mass));
        object_a->angular_velocity_Z += (n.y * point_a.object_com_to_point.x - n.x * point_a.object_com_to_point.y) / object_a->moment_of_inertia * impulse;

        object_b->velocity = object_b->velocity.subtractVector(n.scaleByLength(impulse/object_b->mass));
        object_b->angular_velocity_Z -= (n.y * point_b.object_com_to_point.x - n.x * point_b.object_com_to_point.y) / object_b->moment_of_inertia * impulse;
    }

    else if (object_a->is_fixed == false & object_b->is_fixed == true) {
        object_a->velocity = object_a->velocity.addVector(n.scaleByLength(impulse/object_a->mass));
        object_a->angular_velocity_Z += (n.y * point_a.object_com_to_point.x - n.x * point_a.object_com_to_point.y) / object_a->moment_of_inertia * impulse;
    }

    else if (object_a->is_fixed == true & object_b->is_fixed == false) {
        object_b->velocity = object_b->velocity.subtractVector(n.scaleByLength(impulse/object_b->mass));
        object_b->angular_velocity_Z -= (n.y * point_b.object_com_to_point.x - n.x * point_b.object_com_to_point.y) / object_b->moment_of_inertia * impulse;
    }

    /*
    k_energy_a = object_a->velocity.magnitude() * object_a->velocity.magnitude() * object_a->mass / 2.0;
    gpe_energy_a = object_a->mass * (10.0 + object_a->centre_of_mass.y) * 9.81;
    ang_energy_a = object_a->moment_of_inertia * object_a->angular_velocity_Z * object_a->angular_velocity_Z / 2.0;
    double energy_after_a = k_energy_a + gpe_energy_a + ang_energy_a;

    k_energy_b = object_b->velocity.magnitude() * object_b->velocity.magnitude() * object_b->mass / 2.0;
    gpe_energy_b = object_b->mass * (10.0 + object_b->centre_of_mass.y) * 9.81;
    ang_energy_b = object_b->moment_of_inertia * object_b->angular_velocity_Z * object_b->angular_velocity_Z / 2.0;
    double energy_after_b = k_energy_b + gpe_energy_b + ang_energy_b;

    std::cout << "Energy loss A: " << energy_before_a - energy_after_a << std::endl;
    std::cout << "Energy loss B: " << energy_before_b - energy_after_b << std::endl;
    */
}


void Phys2D::Collision::PointPointDetection() {

    float radii_sum = object_a->radius + object_b->radius;

    // normal is from b to a
    normal = object_a->coord_a.subtractVector(object_b->coord_a);
    distance = normal.magnitude() - radii_sum;

    // too far away
    if (distance > 0) {
        return;
    }

    objects_collide = true;

    point_a = Point();
    point_a.object = object_a;
    point_a.object_com_to_point = normal.invert().scaleToLength(object_a->radius);
    point_a.point = object_a->coord_a.addVector(point_a.object_com_to_point);

    point_b = Point();
    point_b.object = object_b;
    point_b.object_com_to_point = normal.scaleToLength(object_b->radius);
    point_b.point = object_b->coord_a.addVector(point_b.object_com_to_point);
}
