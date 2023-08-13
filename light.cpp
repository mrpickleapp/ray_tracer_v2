#include <iostream>

#include "vector.h"
#include "colour.h"
#include "material.h"
#include "object.h"
#include "intersection.h"
#include "ray.h"
#include "light.h"


Light::Light() {}

Light::Light(int l_id, Colour l_colour, float l_strength, float l_max_angle, int l_func) {
    id = l_id;
    colour = l_colour;
    strength = l_strength;
    max_angle = l_max_angle;
    func = l_func;
}

float Light::incidence(float angle, float max_angle) {
    if (angle > max_angle) {
        return 0;
    }
    if (angle == 0) {
        return 1;
    }
    return ((max_angle - angle) / max_angle);
}

Colour Light::relativeStrength(float angle, float distance) {
    return colour;
}

// =======================================

GlobalLight::GlobalLight() {}

GlobalLight::GlobalLight(int l_id, Vector l_vector, Colour l_colour, float l_strength, float l_max_angle, int l_func):Light(l_id, l_colour, l_strength, l_max_angle, l_func) {
    vector = l_vector.normalise();
}

Colour GlobalLight::relativeStrength(float angle, float distance) {
    if (func == 0) {
        return colour.scaleRGB(incidence(angle, max_angle) * strength);
    }
    return colour;
}


// =========================================

PointLight::PointLight() {}

PointLight::PointLight(int l_id, Vector l_vector, Colour l_colour, float l_strength, float l_max_angle, int l_func):Light(l_id, l_colour, l_strength, l_max_angle, l_func) {
    position = l_vector;
}

Colour PointLight::relativeStrength(float angle, float distance) {
    if (func == -1) {
        return colour.scaleRGB(incidence(angle, max_angle) * strength);
    }
    if (func == 0) {
        return colour.scaleRGB(incidence(angle, max_angle) * strength / distance);
    }
    return colour;
}