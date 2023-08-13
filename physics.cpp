#include <cmath>

#include "physics.h"

#include "vector.h"
#include "colour.h"
#include "material.h"
#include "object.h"
#include "intersection.h"
#include "ray.h"
#include "light.h"
#include "data.h"


Phys::Side* Phys::SidesFromCoords(Vector* _coords, int _n_coords, int _n_sides) {

    Phys::Side* sides = new Phys::Side[_n_sides];

    for (int i = 0; i < _n_sides; ++i) {
        Vector coord_a = _coords[i];
        Vector coord_b = _coords[i + 1 < _n_coords ? i + 1 : 0];
        sides[i] = Side(coord_a, coord_b);
    }

    return sides;
}

Vector Phys::Side::Intersect(Side _side) {

    double s10_x = end.x - start.x;
    double s10_y = end.y - start.y;
    double s32_x = _side.end.x - _side.start.x;
    double s32_y = _side.end.y - _side.start.y;

    double denom = s10_x * s32_y - s32_x * s10_y;

    if (denom == 0) {
        return Vector(false);
    }

    bool denom_is_positive = denom > 0 ? true : false;

    double s02_x = start.x - _side.start.x;
    double s02_y = start.y - _side.start.y;

    double s_numer = s10_x * s02_y - s10_y * s02_x;

    if (s_numer < 0 == denom_is_positive) {
        return Vector(false);
    } 

    double t_numer = s32_x * s02_y - s32_y * s02_x;

    if (t_numer < 0 == denom_is_positive) {
        return Vector(false);
    }

    if (s_numer > denom == denom_is_positive | t_numer > denom == denom_is_positive) {
        return Vector(false);
    }

    double t = t_numer / denom;

    Vector intersect = Vector(start.x + (t * s10_x), start.y + (t * s10_y), 0);

    return intersect;
}



// ============== OBJECT ================== //

/*
- need to make attributes generic to parent class so that
collision handlers can be generic and called on any object
*/


Phys2D::Object::Object() {}

Render::Object* Phys2D::Object::MakeRenderAlias() {
    return NULL;
}

void Phys2D::Object::CalcMass() {}
void Phys2D::Object::CalcMoI() {}

void Phys2D::Object::Accelerate(Sim::Data* sim) {

    if (!is_fixed & has_gravity) {

        // generalise
        float gravity = sim->GRAVITY;
        float tick_ms = sim->TICK_MS;
        int sim_multiple = sim->SIM_MULTIPLE;

        // basic - would need to integrate
        velocity = velocity.addVector(Vector(0, -gravity * 0.001 * tick_ms / sim_multiple, 0));
    }
}

void Phys2D::Object::IteratePhysics(Sim::Data* sim) {}

// ============== CIRCLE ================== //

Phys2D::Circle::Circle() {
    type = CIRCLE;
}

Phys2D::Circle::Circle(int _id, bool _create_alias, Vector _centre, float _radius, Material* _material, Colour _colour) {
    type = CIRCLE;
    id = _id;
    create_alias = _create_alias;
    coord_a = _centre;
    coords = new Vector[1] {_centre};
    n_coords = 1;
    n_sides = 0;
    centre_of_mass = _centre;
    radius = _radius;
    material = _material;
    colour = _colour;
    mass = 4.0/3.0 * 3.14 * radius * radius * radius * material->density;
    moment_of_inertia = (2.0 * mass * radius * radius) / 5.0;
}

Render::Object* Phys2D::Circle::MakeRenderAlias() {
    Render::Sphere* alias = new Render::Sphere(id, coord_a, radius, material, colour);
    return alias;
}

void Phys2D::Circle::IteratePhysics(Sim::Data* sim) {

    Accelerate(sim);

    coord_a = coord_a.addVector(velocity.scaleByLength(0.001 * sim->TICK_MS / sim->SIM_MULTIPLE));

    coords[0] = coords[0].addVector(velocity.scaleByLength(0.001 * sim->TICK_MS / sim->SIM_MULTIPLE));

    centre_of_mass = coords[0];
}


// ============== LINE ================== //

Phys2D::Line::Line() {
    type = LINE;
}

Phys2D::Line::Line(int _id, bool _create_alias, Vector _start, Vector _end, float _radius, Material* _material, Colour _colour) {
    type = LINE;
    id = _id;
    create_alias = _create_alias;
    coord_a = _start;
    coord_b = _end;
    coords = new Vector[2] {_start, _end};
    n_coords = 2;
    sides = Phys::SidesFromCoords(coords, n_coords, 1);
    n_sides = 1;
    centre_of_mass = sides[0].start.addVector(sides[0].vector.scaleByLength(0.5));
    pivot = centre_of_mass;
    radius = _radius;
    material = _material;
    colour = _colour;
    length = coord_b.subtractVector(coord_b).magnitude();
    CalcMass();
    CalcMoI();

    coords_to_com = new Vector[n_coords];
    for (int i = 0; i < n_coords; ++i) {
        coords_to_com[i] = coords[i].subtractVector(centre_of_mass);
    }
}

void Phys2D::Line::CalcMass() {
    mass = 3.14 * radius * radius * coord_a.distanceFrom(coord_b) * material->density;
}

void Phys2D::Line::CalcMoI() {
    moment_of_inertia = (0.25 * mass * radius * radius) + (1.0/12.0 * mass * length * length);
}


Render::Object* Phys2D::Line::MakeRenderAlias() {
    Render::Cylinder* alias = new Render::Cylinder(id, coords[0], coords[1], radius, material, colour);
    return alias;
}



void Phys2D::Line::IteratePhysics(Sim::Data* sim) {

    Accelerate(sim);

    centre_of_mass = centre_of_mass.addVector(velocity.scaleByLength(0.001 * sim->TICK_MS / sim->SIM_MULTIPLE));
    pivot = centre_of_mass;

    for (int i = 0; i < n_coords; ++i) {
        double rotation_inc = angular_velocity_Z * 0.001 * sim->TICK_MS / sim->SIM_MULTIPLE;
        coords_to_com[i] = coords_to_com[i].rotateExtrinsic(0, 0, rotation_inc);
        coords[i] = centre_of_mass.addVector(coords_to_com[i]);
    }

    // update vertices
    delete [] sides;
    sides = Phys::SidesFromCoords(coords, n_coords, 1);
}


// ============= POLYGON ===============//

Phys2D::Polygon::Polygon() {}

Phys2D::Polygon::Polygon(int _id, bool _create_alias, Vector _com, float _moi, Vector* _coords, int _n_coords, Material* _material, Colour _colour, double _thickness) {
    type = POLYGON;
    id = _id;
    create_alias = _create_alias;
    centre_of_mass = _com;
    pivot = _com;
    moment_of_inertia = _moi;
    coords = _coords;
    n_coords = _n_coords;
    sides = Phys::SidesFromCoords(coords, n_coords, n_coords);
    n_sides = _n_coords;
    material = _material;
    colour = _colour;
    thickness = _thickness;

    coords_to_com = new Vector[n_coords];
    for (int i = 0; i < n_coords; ++i) {
        coords_to_com[i] = coords[i].subtractVector(centre_of_mass);
    }
    
}

Render::Object* Phys2D::Polygon::MakeRenderAlias() {
    Render::Prism* alias = new Render::Prism(id, coords, n_coords, material, colour, thickness);
    return alias;
}

void Phys2D::Polygon::IteratePhysics(Sim::Data* sim) {

    Accelerate(sim);

    centre_of_mass = centre_of_mass.addVector(velocity.scaleByLength(0.001 * sim->TICK_MS / sim->SIM_MULTIPLE));

    // if on pivot, move centre of mass
    if (has_pivot) {
        Vector pivot_to_com = pivot.subtractVector(centre_of_mass);
        double pivot_radius = pivot_to_com.magnitude();
        double angle = 4.71239 - pivot_to_com.angleBetween(Vector(1, 0, 0));

        double force = sin(angle) * sim->GRAVITY;
        double torque = pivot_radius * force;

        double acc = (torque / moment_of_inertia) * (0.001 * sim->TICK_MS / sim->SIM_MULTIPLE);
        angular_velocity_Z -= acc;

        // rotate centre of mass around pivot
        double rotation_inc = angular_velocity_Z * (0.001 * sim->TICK_MS / sim->SIM_MULTIPLE);
        centre_of_mass = pivot.subtractVector(pivot_to_com.rotateExtrinsic(0, 0, rotation_inc));

        // update points
        for (int i = 0; i < n_coords; ++i) {
            coords_to_com[i] = coords_to_com[i].rotateExtrinsic(0, 0, rotation_inc);
            coords[i] = centre_of_mass.addVector(coords_to_com[i]);
        }

    }

    // else, move pivot to centre of mass
    else {
        for (int i = 0; i < n_coords; ++i) {

            double acc = (tick_torque / moment_of_inertia) * (0.001 * sim->TICK_MS / sim->SIM_MULTIPLE);
            angular_velocity_Z -= acc;
            
            double rotation_inc = angular_velocity_Z * 0.001 * sim->TICK_MS / sim->SIM_MULTIPLE;
            coords_to_com[i] = coords_to_com[i].rotateExtrinsic(0, 0, rotation_inc);
            coords[i] = centre_of_mass.addVector(coords_to_com[i]);

            pivot = centre_of_mass;
        }
    }

    // update vertices
    delete [] sides;
    sides = Phys::SidesFromCoords(coords, n_coords, n_coords);

    // need to calculate centre of mass, rotation, etc
    // need to update centre of mass
}


// ============= GEAR ===============//

Phys2D::Polygon* Phys2D::MakeGear(int _id, bool _create_alias, Vector _com, Vector _tooth_vector, int _n_teeth, double _radius, Material* _material, Colour _colour, double _thickness) {

    int n_points = 2 * _n_teeth;

    Vector point_0_vector = Vector(0, _radius, 0);
    Vector point_1_vector = point_0_vector.addVector(_tooth_vector);

    double theta = 6.28319 / _n_teeth;

    Vector* gear_points = new Vector[n_points];

    int tooth = 0;
    for (int i = 0; i < n_points; i += 2) {

        gear_points[i] = _com.addVector(point_1_vector.rotateExtrinsic(0, 0, theta * tooth));
        gear_points[i + 1] = _com.addVector(point_0_vector.rotateExtrinsic(0, 0, theta * tooth));

        tooth += 1;
    }

    // TEMP
    double _moi = 1;

    Phys2D::Polygon *gear = new Phys2D::Polygon(_id, _create_alias, _com, _moi, gear_points, n_points, _material, _colour, _thickness);

    return gear;

}

