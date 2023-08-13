#pragma once

#include "vector.h"
#include "colour.h"
#include "material.h"
#include "object.h"
#include "intersection.h"
#include "ray.h"
#include "light.h"
#include "data.h"


namespace Phys2D {class Object;}
namespace Sim {class Data;}
class Vertex;


namespace Phys2D {

    class Point {
        public:
            Phys2D::Object* object = nullptr;
            Vector point;
            Vector object_com_to_point;
            double point_angular_v;
            Vector point_rotation_vector;
            Vector point_velocity;

            Point();
            Vector PointVelocity();
    };
    
    class Collision {
        public:
            bool objects_collide = false;
            bool objects_in_contact = false;
            Phys2D::Object* object_a;
            Phys2D::Object* object_b;
            Vector point;

            Point point_a;
            Point point_b;

            Vector relative_point_velocity;
            Vector normal;
            double distance = 1e10;
            double approach_factor;
            bool objects_converging = false;

            Collision();
            Collision(Phys2D::Object*, Phys2D::Object*);

            // get point a -> from object?
            // get point b -> from object?

            // get distance between points

            void ClosestPoints();
            void PointPointDetection();
            void PointLineDetection(int);

            void CircleLineDetection();
            int LineLineDetection();
            
            Vector RelativePointVelocity();
            bool PointsConverging();

            void Resolve(Sim::Data*);
    };

    // Collision GetNearestPoints
    // Generalise
    /*
        - circle is always just one point
        - line is one vertex / two vertices

        - For any shape, we need to
            - compare points to either one point, or a list of vertices
    */

    Collision CircleCircleCollision(Phys2D::Object*, Phys2D::Object*, Sim::Data*);

    Collision CircleLineCollision(Phys2D::Object*, Phys2D::Object*);

    Collision CirclePolyCollision(Phys2D::Object*, Phys2D::Object*, Sim::Data*);

    // Generalised collision
    /*
        - Find closest points between objects
        - Get 
        - Determine if converging
        - Resolve 
    */
}