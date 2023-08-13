#pragma once

#include "vector.h"
#include "colour.h"
#include "material.h"
#include "object.h"
#include "intersection.h"
#include "ray.h"
#include "light.h"

class Colour;


class Light {
    public:
        int id;
        Colour colour;
        float strength;
        float max_angle;
        int func;

        Light();
        Light(int, Colour, float, float, int func=0);

        float incidence(float, float);

        virtual Colour relativeStrength(float, float);
};


class GlobalLight: public Light {
    public:
        Vector vector;
        
        GlobalLight();
        GlobalLight(int, Vector, Colour, float, float, int);

        Colour relativeStrength(float, float);
};


class PointLight: public Light {
    public:
        Vector position;

        PointLight();
        PointLight(int, Vector, Colour, float, float, int);

        Colour relativeStrength(float, float);
};