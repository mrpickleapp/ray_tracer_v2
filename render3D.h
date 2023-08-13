#include "vector.h"
#include "colour.h"
#include "material.h"
#include "object.h"
#include "intersection.h"
#include "ray.h"
#include "light.h"

#include "stb_image_write.h"

namespace render3D {

    int render(
        const char*,
        Object*[],
        int,
        PointLight[],
        int,
        GlobalLight[],
        int,
        int=400,
        int=300,
        float=1,
        float=5,
        float=0,
        float=1,
        int=20
    );
}