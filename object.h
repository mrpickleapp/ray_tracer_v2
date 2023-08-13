#pragma once

#include "vector.h"
#include "colour.h"
#include "material.h"
#include "object.h"
#include "intersection.h"
#include "ray.h"
#include "light.h"
#include "physics.h"

class Ray;
class Material;
class Colour;
class Intersection;
namespace Phys2D {class Object;}

namespace Render {

    class Object {
        public:
            int id;

            Material* material_ptr;
            Colour colour;

            Object();
            Object(int, Material*, Colour);

            virtual Intersection intersection(Ray*);
            virtual double isInside(Vector);
    };


    class Sphere: public Object {
        public:
            Vector centre;
            double radius;

            Sphere(int, Vector, double, Material*, Colour);

            Intersection intersection(Ray*);
            double isInside(Vector);
    };


    class Disc: public Object {
        public:
            Vector centre;
            double radius;
            double radius_2;
            Vector normal;
            Vector normal_inv;

            Disc();
            Disc(int, Vector, double, Vector, Material*, Colour);

            Intersection intersection(Ray*);
            double isInside(Vector);
    };


    class Polygon: public Object {
        public:
            Vector* vertices;
            int n_vertices;
            Vector normal;
            Vector normal_inv;
            double thickness = 0;

            Polygon();
            Polygon(int, Vector*, int, Material*, Colour, double=0);

            Intersection intersection(Ray*);
            double isInside(Vector);
    };

    class Prism: public Object {
        public:
            Render::Polygon poly_1;
            Render::Polygon poly_2;

            Object* components;

            Prism();
            Prism(int, Vector*, int, Material*, Colour, double);

            Intersection intersection(Ray*);
            double isInside(Vector);

    };

    class Pyramid: public Object {
        public:
            Vector* vertices;
            Render::Polygon side_a;
            Render::Polygon side_b;
            Render::Polygon side_c;
            Render::Polygon side_d;
            Object* components[4];

            Pyramid();
            Pyramid(int, Vector*, Material*, Colour);

            Intersection intersection(Ray*);
            double isInside(Vector);
    };

    class FacetedDiamond: public Object {
        public:
            Vector top;
            Vector bottom;
            int n_sides;
            Object* components[100];
            int n_components;

            FacetedDiamond();
            FacetedDiamond(int, Vector _start, Vector _end, int _n_sides, Material*, Colour);

            Intersection intersection(Ray*);
            double isInside(Vector);

    };

    class Pipe: public Object {
        public:
            Vector start_centre;
            Vector end_centre;
            double radius;
            Vector axis;
            double baba;

            Pipe();
            Pipe(int, Vector, Vector, double, Material*, Colour);

            Intersection intersection(Ray*);
            double isInside(Vector);
    };

    class Cylinder: public Object {
        public:
            Pipe pipe;
            Disc start_disc;
            Disc end_disc;
            Object* components[3];

            Cylinder(int, Vector, Vector, double, Material*, Colour);

            Intersection intersection(Ray*);
            double isInside(Vector);
    };


    class Square: public Object {
        public:
            Vector root;
            Vector normal;
            Vector normal_inv;
            Vector side_A;
            Vector side_B;
            double side_A_length;
            double side_B_length;

            Square();
            Square(int, Vector, Vector, Vector, Material*, Colour);

            Intersection intersection(Ray*);
            double isInside(Vector);
    };
}