#include <iostream>
#include <cmath>

#include "vector.h"
#include "colour.h"
#include "material.h"
#include "object.h"
#include "intersection.h"
#include "ray.h"
#include "light.h"
#include "physics.h"

namespace Phys {class Side;}


//============= OBJECT ===============//

Render::Object::Object() {
    id = -1;
}

Render::Object::Object(int o_id, Material* o_material_ptr, Colour o_colour) {
    id = o_id;
    material_ptr = o_material_ptr;
    colour = o_colour;
}

Intersection Render::Object::intersection(Ray* ray) {
    return Intersection();
}

double Render::Object::isInside(Vector v) {
    return -1;
}

//============= SPHERE ===============//

Render::Sphere::Sphere(int o_id, Vector o_centre, double o_radius, Material* o_material_ptr, Colour o_colour):Render::Object(o_id, o_material_ptr, o_colour) {
    centre = o_centre;
    radius = o_radius;
    material_ptr = o_material_ptr;
    colour = o_colour;
}

Intersection Render::Sphere::intersection(Ray* ray) {

    Vector L = centre.subtractVector(ray->origin);

    double tca = L.dotProduct(ray->D);

    if (tca < 0) {
        return Intersection();
    }

    double d = sqrt((L.dotProduct(L) - tca*tca));
    if (std::isnan(d)) {
        d = 0;
    }

    if (d > radius) {
        return Intersection();
    }

    double thc = sqrt(radius*radius - d*d);
    double t0 = tca - thc;
    double t1 = tca + thc;

    bool is_entering = true;
    double tmin = t0;
    if (t0 < 1e-6) {
        tmin = t1;
        is_entering = false;
    }

    Vector phit = ray->origin.addVector(ray->D.scaleByLength(tmin));
    Vector nhit = phit.subtractVector(centre).normalise();

    return Intersection(
        true,
        is_entering,
        tmin,
        phit,
        nhit,
        this
    );
}

double Render::Sphere::isInside(Vector point) {

    double d = centre.distanceFrom(point);
    if (d < radius) {
        return radius;
    }
    return -1;    
}


//============= DISC ===============//

Render::Disc::Disc() {}

Render::Disc::Disc(int o_id, Vector o_centre, double o_radius, Vector o_normal, Material* o_material_ptr, Colour o_colour):Render::Object(o_id, o_material_ptr, o_colour) {
    centre = o_centre;
    radius = o_radius;
    radius_2 = radius * radius;
    normal = o_normal;
    normal_inv = normal.invert();
}

Intersection Render::Disc::intersection(Ray* ray) {

    double denom = ray->D.dotProduct(normal);

    if (denom == 0) {
        return Intersection();
    }

    // ray is entering from side of normal
    if (denom < 0) {
        Vector p = centre.subtractVector(ray->origin);
        double t = p.dotProduct(normal) / denom;

        if (t > 1e-10) {
            p = ray->origin.addVector(ray->D.scaleByLength(t));
            Vector v = p.subtractVector(centre);
            double d2 = v.dotProduct(v);

            if (d2 < radius_2) {
                return Intersection(true, true, t, p, normal, this);
            }
        }
    }

    // ray is entering from side of normal inverted
    denom = -denom;

    if (denom < 0) {
        Vector p = centre.subtractVector(ray->origin);
        double t = p.dotProduct(normal_inv) / denom;

        if (t > 1e-10) {
            p = ray->origin.addVector(ray->D.scaleByLength(t));
            Vector v = p.subtractVector(centre);
            double d2 = v.dotProduct(v);

            if (d2 < radius_2) {
                return Intersection(true, false, t, p, normal_inv, this);
            }
        }
    }

    return Intersection();
}

double Render::Disc::isInside(Vector point) {
    return -1;
}


//============ POLYGON ============//

// Polygons are always drawn anticlockwise
// Only works when Z=0?
// Doesn't work when polygon is planar with z axis
// Probably need to rotate it to Z=0 to check properly

Render::Polygon::Polygon() {}

Render::Polygon::Polygon(int _id, Vector* _vertices, int _n_vertices, Material* _material, Colour _colour, double _thickness):Render::Object(_id, _material, _colour) {
    vertices = _vertices;
    n_vertices = _n_vertices;
    normal = vertices[1].subtractVector(vertices[0]).crossProduct(vertices[2].subtractVector(vertices[1])).normalise();
    normal_inv = normal.invert();
    thickness = _thickness;
}

// https://towardsdatascience.com/is-the-point-inside-the-polygon-574b86472119#:~:text=Algorithm%3A%20For%20a%20convex%20polygon,segments%20making%20up%20the%20path.
Intersection Render::Polygon::intersection(Ray* ray) {

    Vector norm = normal;
    double denom = ray->D.dotProduct(normal);

    if (denom == 0) {
        return Intersection();
    }


    // once for each side
    for (int s = 0; s < 2; ++s) {

        if (s == 1) {
            denom = -denom;
            norm = normal_inv;
        }

        if (denom < 0) {

            Vector p = vertices[0].subtractVector(ray->origin);
            double t = p.dotProduct(norm) / denom;

            int winding_number = 0;

            if (t > 1e-10) {
                p = ray->origin.addVector(ray->D.scaleByLength(t));
                Vector v = p.subtractVector(vertices[0]);
                double d2 = v.dotProduct(v);

                for (int i = 0; i < n_vertices; ++i) {

                    double point_in_line = p.inLine(vertices[i], vertices[(i + 1) % n_vertices]);

                    // downwards crossing
                    if (point_in_line == 0) {
                        Intersection(true, true, t, p, norm, this);
                    }

                    // upwards crossing
                    if (vertices[i].y < p.y) {
                        if (vertices[(i + 1) % n_vertices].y > p.y) {
                            if (point_in_line > 0) {
                                ++winding_number;
                            }
                        }
                    }

                    // downwards crossing
                    else if (vertices[(i + 1) % n_vertices].y < p.y) {
                        if (point_in_line < 0) {
                            --winding_number;
                        }
                    }
                }

                // point is inside
                if (winding_number != 0) {
                    return Intersection(true, true, t, p, norm, this);
                }

                /*
                // check if on polygon edge
                Vector seek_line_start = p.subtractVector(norm.scaleToLength(thickness));

                Vector seek_vector = ray->D.crossProduct(norm).crossProduct(ray->D);
                
                Vector seek_line_end = seek_line_start.addVector(seek_vector);            
                Phys::Side seek_line = Phys::Side(seek_line_start.nullZ(), seek_line_end.nullZ());

                bool side_is_hit = false;
                double min_seek_dist = 1e10;
                Vector side_intersect;
                Phys::Side hit_side;

                for (int i = 0; i < n_vertices; ++i) {

                    Phys::Side side = Phys::Side(vertices[i].nullZ(), vertices[(i + 1) % n_vertices].nullZ());

                    Vector intersect = seek_line.Intersect(side);

                    if (intersect.valid) {

                        side_is_hit = true;

                        double seek_dist = seek_line.start.subtractVector(intersect).magnitude();

                        if (seek_dist < min_seek_dist) {
                            min_seek_dist = seek_dist;
                            side_intersect = intersect;
                            hit_side = side;
                        }
                    }
                }

                if (side_is_hit) {

                    p = side_intersect.subtractVector(seek_vector.scaleToLength(seek_line.start.subtractVector(side_intersect).magnitude()));

                    t = p.subtractVector(ray->origin).magnitude();

                    return Intersection(true, true, t, p, ray->D.crossProduct(hit_side.vector), this);
                }
                */
            }
        }
    }

    return Intersection();
}

double Render::Polygon::isInside(Vector point) {
    return -1;
}

//============= PRISM ==============//

Render::Prism::Prism() {}

Render::Prism::Prism(int _id, Vector* _vertices, int _n_vertices, Material* _material, Colour _colour, double _thickness) {
    id = _id;

    Vector* poly_1_vertices = _vertices;
    Vector* poly_2_vertices = new Vector[_n_vertices];

    for (int i = 0; i < _n_vertices; ++i) {

        // assuming prism length is Z only

        poly_2_vertices[i] = poly_1_vertices[i].addVector(Vector(0, 0, -0.5));
    }

    poly_1 = Render::Polygon(_id, poly_1_vertices, _n_vertices, _material, _colour);
    poly_2 = Render::Polygon(_id, poly_2_vertices, _n_vertices, _material, _colour);


    components = new Render::Polygon[2];

    components[0] = poly_1;
    components[1] = poly_2;
}

Intersection Render::Prism::intersection(Ray* ray) {
    return ray->nearestIntersect(&components, 2);
}

double Render::Prism::isInside(Vector point) {
    return -1;
}



//============= PYRAMID ============//

Render::Pyramid::Pyramid() {}

Render::Pyramid::Pyramid(int _id, Vector* _vertices, Material* _material, Colour _colour) {
    id = _id;
    
    Vector* side_a_vertices = new Vector[3]{_vertices[0], _vertices[1], _vertices[2]};
    Vector* side_b_vertices = new Vector[3]{_vertices[0], _vertices[2], _vertices[3]};
    Vector* side_c_vertices = new Vector[3]{_vertices[0], _vertices[3], _vertices[1]};
    Vector* side_d_vertices = new Vector[3]{_vertices[3], _vertices[2], _vertices[1]};
    
    side_a = Render::Polygon(id, side_a_vertices, 3, _material, _colour);
    side_b = Render::Polygon(id, side_b_vertices, 3, _material, _colour);
    side_c = Render::Polygon(id, side_c_vertices, 3, _material, _colour);
    side_d = Render::Polygon(id, side_d_vertices, 3, _material, _colour);

    components[0] = &side_a;
    components[1] = &side_b;
    components[2] = &side_c;
    components[3] = &side_d;
}

Intersection Render::Pyramid::intersection(Ray* ray) {
    return ray->nearestIntersect(components, 4);
}

double Render::Pyramid::isInside(Vector point) {

    // TO DO 
    return -1;
}



//=========== FACETED DIAMOND ======//

Render::FacetedDiamond::FacetedDiamond() {}

Render::FacetedDiamond::FacetedDiamond(int _id, Vector _top, Vector _bottom, int _n_sides, Material* _material, Colour _colour) {

    id = _id;
    top = _top;
    bottom = _bottom;
    n_sides = _n_sides;
    n_components = 0;

    float full_turn = 6.28319;
    float theta = full_turn / n_sides;

    Render::Polygon* facets[100];

    // Layer 1
    float height_1 = 0.1;
    float width_1 = 1.5;
    Vector base_side_vector = top.subtractVector(Vector(0, height_1, width_1)).rotateExtrinsic(0, -theta/2, 0);

    for (int i = 0; i < n_sides; ++i) {
        Vector *vertices = new Vector[3] {
            top, 
            base_side_vector.rotateExtrinsic(0, theta*i, 0),
            base_side_vector.rotateExtrinsic(0, theta + theta*i, 0)
        };

        Render::Polygon* facet = new Render::Polygon(id, vertices, 3, _material, _colour);
        components[n_components] = facet;
        facets[n_components] = facet;
        n_components++;
    }

    // Layer 2
    float height_2 = 0.5;
    float width_2 = 2;
    base_side_vector = top.subtractVector(Vector(0, height_2, width_2)).rotateExtrinsic(0, 0, 0);

    for (int i = 0; i < n_sides; ++i) {
        Render::Polygon* above_facet = facets[i];
        Vector *vertices = new Vector[3] {
            above_facet->vertices[1],
            base_side_vector.rotateIntrinsic(0, theta*i, 0),
            above_facet->vertices[2]
        };
        Render::Polygon* facet = new Render::Polygon(id, vertices, 3, _material, _colour);
        components[n_components] = facet;
        facets[n_components] = facet;
        n_components++;
    }

    // Layer 3
    Vector base_top_vector = top.subtractVector(Vector(0, height_1, width_1)).rotateExtrinsic(0, theta/2, 0);
    Vector base_left_vector = top.subtractVector(Vector(0, height_2, width_2)).rotateExtrinsic(0, 0, 0);
    
    for (int i = 0; i < n_sides; ++i) {
        Vector *vertices = new Vector[3] {
            base_top_vector.rotateExtrinsic(0, theta * i, 0),
            base_left_vector.rotateExtrinsic(0, theta * i, 0),
            base_left_vector.rotateExtrinsic(0, theta * i + theta, 0)
        };
        Render::Polygon* facet = new Render::Polygon(id, vertices, 3, _material, _colour);
        components[n_components] = facet;
        facets[n_components] = facet;
        n_components++;
    }

    // Layer 4
    base_top_vector = top.subtractVector(Vector(0, height_2, width_2)).rotateExtrinsic(0, 0, 0);

    for (int i = 0; i < n_sides; ++i) {
        Vector *vertices = new Vector[3] {
            base_top_vector.rotateExtrinsic(0, theta * i, 0),
            bottom,
            base_top_vector.rotateExtrinsic(0, theta * i + theta, 0)
        };

        Render::Polygon* facet = new Render::Polygon(id, vertices, 3, _material, _colour);
        components[n_components] = facet;
        facets[n_components] = facet;
        n_components++;
    }

}

Intersection Render::FacetedDiamond::intersection(Ray* ray) {
    return ray->nearestIntersect(components, n_components);
}

double Render::FacetedDiamond::isInside(Vector point) {

    // TO DO
    return -1;
}


//============= PIPE ===============//

Render::Pipe::Pipe() {}

Render::Pipe::Pipe(int o_id, Vector o_start_centre, Vector o_end_centre, double o_radius, Material* o_material_ptr, Colour o_colour):Render::Object(o_id, o_material_ptr, o_colour) {
    start_centre = o_start_centre;
    end_centre = o_end_centre;
    radius = o_radius;
    axis = end_centre.subtractVector(start_centre);
    baba = axis.dotProduct(axis);
}

Intersection Render::Pipe::intersection(Ray* ray) {

    // ra = radius
    // ba = axis
    // rd = ray->D

    Vector oc = ray->origin.subtractVector(start_centre);

    double bard = axis.dotProduct(ray->D);
    double baoc = axis.dotProduct(oc);
    double k2 = baba - bard*bard;

    if (k2 == 0) {
        return Intersection();
    }

    double k1 = baba * oc.dotProduct(ray->D) - baoc*bard;
    double k0 = baba * oc.dotProduct(oc) - baoc*baoc - radius*radius*baba;
    double h = k1*k1 - k2*k0;

    if (h < 0) {
        return Intersection();
    }

    h = sqrt(h);
    double t0 = (-k1-h)/k2;
    double t1 = (-k1+h)/k2;

    // first intersection
    double y = baoc + t0*bard;
    if (y > 0 && y < baba && t0 > 1e-10) {
        Vector phit = ray->origin.addVector(ray->D.scaleByLength(t0));
        Vector nhit = oc.addVector(ray->D.scaleByLength(t0).subtractVector(axis.scaleByLength(y/baba))).scaleByLength(1/radius);
        return Intersection(true, true, t0, phit, nhit, this);
    }

    // second intersection
    y = baoc + t1*bard;
    if (y > 0 && y < baba && t1 > 1e-10) {
        Vector phit = ray->origin.addVector(ray->D.scaleByLength(t1));
        Vector nhit = oc.addVector(ray->D.scaleByLength(t1).subtractVector(axis.scaleByLength(y/baba))).scaleByLength(1/radius);
        return Intersection(true, false, t1, phit, nhit, this);
    }

    return Intersection();
}

double Render::Pipe::isInside(Vector point) {
    return -1;
}


//============= CYLINDER ===============//

Render::Cylinder::Cylinder(int o_id, Vector o_start_centre, Vector o_end_centre, double o_radius, Material* o_material_ptr, Colour o_colour):Render::Object(o_id, o_material_ptr, o_colour) {
    pipe = Pipe(o_id, o_start_centre, o_end_centre, o_radius, o_material_ptr, o_colour);
    start_disc = Disc(o_id, o_start_centre, o_radius, pipe.axis.invert(), o_material_ptr, o_colour);
    end_disc = Disc(o_id, o_end_centre, o_radius, pipe.axis, o_material_ptr, o_colour);
    components[0] = &pipe;
    components[1] = &start_disc;
    components[2] = &end_disc;
}

Intersection Render::Cylinder::intersection(Ray* ray) {
    return ray->nearestIntersect(components, 3);
}

double Render::Cylinder::isInside(Vector point) {
    // distance to axis
    double d = pipe.start_centre.subtractVector(point).crossProduct(pipe.axis).magnitude();
    if (d > pipe.radius) {
        return -1;
    }

    // distance to start end
    d = pipe.start_centre.subtractVector(point).dotProduct(start_disc.normal);
    if (d < 0) {
        return -1;
    }

    // distance to end end
    d = pipe.end_centre.subtractVector(point).dotProduct(start_disc.normal);
    if (d < 0) {
        return -1;
    }

    return pipe.radius;
}


//============= SQUARE ===============//

Render::Square::Square() {}

Render::Square::Square(int o_id, Vector o_root, Vector o_side_A, Vector o_side_B, Material* o_material_ptr, Colour o_colour):Render::Object(o_id, o_material_ptr, o_colour) {
    root = o_root;
    side_A = o_side_A;
    side_B = o_side_B;
    side_A_length = side_A.magnitude();
    side_B_length = side_B.magnitude();
    normal = side_A.crossProduct(side_B).normalise();
    normal_inv = normal.invert();
}

Intersection Render::Square::intersection(Ray* ray) {

    double denom = ray->D.dotProduct(normal);

    if (denom == 0) {
        return Intersection();
    }

    // ray is entering from side of normal
    if (denom < 0) {
        Vector p = root.subtractVector(ray->origin);
        double t = p.dotProduct(normal) / denom;

        if (t > 1e-10) {
            p = ray->origin.addVector(ray->D.scaleByLength(t));
            Vector v = p.subtractVector(root);
            double q1 = v.dotProduct(side_A) / side_A_length;
            double q2 = v.dotProduct(side_B) / side_B_length;

            if (q1 >= 0 && q1 <= side_A_length && q2 >=0 && q2 <= side_B_length) {
                return Intersection(true, true, t, p, normal, this);
            }
        }
    }

    // ray is entering from side of normal inverted
    denom = -denom;

    if (denom < 0) {
        Vector p = root.subtractVector(ray->origin);
        double t = p.dotProduct(normal_inv) / denom;

        if (t > 1e-10) {
            p = ray->origin.addVector(ray->D.scaleByLength(t));
            Vector v = p.subtractVector(root);
            double q1 = v.dotProduct(side_A) / side_A_length;
            double q2 = v.dotProduct(side_B) / side_B_length;

            if (q1 >= 0 && q1 <= side_A_length && q2 >=0 && q2 <= side_B_length) {
                return Intersection(true, false, t, p, normal_inv, this);
            }
        }
    }

    return Intersection();
}

double Render::Square::isInside(Vector point) {
    return -1;
}






