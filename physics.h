#pragma once

#include <limits>

#include "vector.h"
#include "colour.h"
#include "material.h"
#include "object.h"
#include "intersection.h"
#include "ray.h"
#include "light.h"
#include "data.h"
#include "collision.h"

class Material;
class Colour;
namespace Render {class Object;}
namespace Sim {class Data;}
namespace Phys2D {class Collision;}

#define OBJECT 0
#define CIRCLE 1
#define LINE 2
#define POLYGON 3


namespace Phys {

    class Side {
        public:
            Vector start;
            Vector end;
            Vector vector;
            double length;

            Side() {};
            Side(Vector _start, Vector _end) {
                start = _start;
                end = _end;
                vector = end.subtractVector(start);
                length = vector.magnitude();
            }

            Vector Intersect(Side);
    };

    Side* SidesFromCoords(Vector*, int, int);
}


namespace Phys2D {

    class Object {
        public:
            int type = OBJECT;
            int id;
            bool create_alias;
            Render::Object* render_alias;
            Material* material;
            Colour colour;

            bool has_gravity = true;
            bool has_pivot = false;
            bool is_fixed = false;
            bool can_collide = true;
            double mass;
            double moment_of_inertia;

            Vector centre_of_mass;
            Vector pivot;

            Vector coord_a;
            Vector coord_b;
            Vector coord_c;
            Vector* coords;
            Vector* coords_to_com;
            int n_coords;

            Phys::Side* sides;
            int n_sides = 0;

            double radius = 0;
            double thickness = 0;

            Vector velocity = Vector(0, 0, 0);
            double angular_velocity_Z = 0;
            double rotation_Z = 0;

            double tick_torque = 0;

            Object();

            virtual void CalcMass();
            virtual void CalcMoI();

            void Accelerate(Sim::Data*);

            virtual Render::Object* MakeRenderAlias();
            virtual void IteratePhysics(Sim::Data*);

    };

    class Circle: public Object {
        public:
            Circle();
            Circle(int, bool, Vector, float, Material*, Colour);

            Render::Object* MakeRenderAlias();
            void IteratePhysics(Sim::Data*);
    };

    class Line: public Object {
        public:
            float length;
            Line();
            Line(int, bool, Vector, Vector, float, Material*, Colour);

            void CalcMass();
            void CalcMoI();

            Render::Object* MakeRenderAlias();
            void IteratePhysics(Sim::Data*);
    };

    class Polygon: public Object {
        public:
            Polygon();
            Polygon(int, bool, Vector, float, Vector*, int, Material*, Colour, double=0);

            Render::Object* MakeRenderAlias();
            void IteratePhysics(Sim::Data*);
    };

    Phys2D::Polygon* MakeGear(int, bool, Vector, Vector, int, double, Material*, Colour, double=0);

}


namespace Phys3D {

    class Object {

    };
}