#pragma once

#include "vector.h"
#include "colour.h"
#include "material.h"
#include "object.h"
#include "intersection.h"
#include "ray.h"
#include "light.h"


namespace Render {class Object;}
class Colour;
class Light;

class Intersection{
    public:
        bool intersects, is_entering;
        double distance;
        Vector point, normal;
        Render::Object* object_ptr;

        Intersection();
        Intersection(bool, bool, double, Vector, Vector, Render::Object*);

        Colour directionRGB(Colour, GlobalLight*, int);
        Colour terminalRGB(Colour, Render::Object*[], int, GlobalLight*, int, PointLight*, int, int=0);
};