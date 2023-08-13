#pragma once

#include <iostream>

class Colour {
    public:
        float r, g, b;

        Colour() {}

        Colour(float cr, float cg, float cb) {
            r = cr;
            g = cg;
            b = cb;
        }

        void describe() {
            std::cout << "r: " << r << ", g: " << g << ", b: " << b << std::endl;
        }

        Colour round() {
            return Colour(int(r), int(g), int(b));
        }

        Colour addColour(Colour B) {
            return Colour(r + B.r, g + B.g, b + B.b);
        }

        Colour scaleRGB(float s) {
            return Colour(r * s, g * s, b * s);
        }

        Colour illuminate(Colour light) {
            float r_factor = (light.r>0)?light.r/255:0;
            float g_factor = (light.g>0)?light.g/255:0;
            float b_factor = (light.b>0)?light.b/255:0;
            return Colour(r * r_factor, g * g_factor, b * b_factor);
        }

        Colour ceil(int n=255) {
            // return Colour(r, g, b);
            return Colour((r>n)?n:r, (g>n)?n:g, (b>n)?n:b);
        }

        Colour avg(Colour colour_b, float weight_a, float weight_b) {
            float weight_sum = weight_a + weight_b;
            float new_r = (r*weight_a + colour_b.r*weight_b) / weight_sum;
            float new_g = (g*weight_a + colour_b.g*weight_b) / weight_sum;
            float new_b = (b*weight_a + colour_b.b*weight_b) / weight_sum;
            return Colour(new_r, new_g, new_b);
        }
};


/*

class Colour():

    def getList(self):
        return [self.r, self.g, self.b]

*/