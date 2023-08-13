#pragma once

#include "vector.h"
#include "colour.h"
#include "material.h"
#include "object.h"
#include "intersection.h"
#include "ray.h"
#include "light.h"


namespace Render {class Object;}
class Intersection;

class Ray {
    public:
        Vector origin, D;
        float current_ri;

        Ray();
        Ray(Vector, Vector, float=1);

        Intersection nearestIntersect(Render::Object*[], int, int=-1);
        Colour objectColour(Render::Object*[], int, Colour);
        Colour shadedColour(Render::Object*[], int, Colour);
        Colour aggColour(Render::Object*[], int, Colour, GlobalLight*, int, PointLight*, int, int=0, int=10, int=1, int=-1);
        Render::Object* containedBy(Vector, Render::Object*[], int, int);
};