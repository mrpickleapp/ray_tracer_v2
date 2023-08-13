#pragma once

#include <iostream>


class Material {
    public:
        bool reflective, transparent, emitive;
        float refractive_index;
        float reflection_ratio;
        float density;

        Material() {
            reflective = false;
            transparent = false;
            emitive = false;
            refractive_index = 1;
            reflection_ratio = 1;
            density = 1;
        }

        Material(bool m_reflective, bool m_transparent, bool m_emitive, float m_refractive_index, float m_reflection_ratio=0, float m_density=1) {
            reflective = m_reflective;
            transparent = m_transparent;
            emitive = m_emitive;
            refractive_index = m_refractive_index;
            reflection_ratio = m_reflection_ratio;
            density = m_density;
        }
};