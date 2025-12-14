#ifndef MATERIAL_H
#define MATERIAL_H

#include "imageLoader.h"
#include "Vec3.h"
#include <cmath>

#include <GL/glut.h>

enum MaterialType
{
    Material_Diffuse_Blinn_Phong,
    Material_Glass,
    Material_Mirror,
    Material_Water
};

struct Material
{
    Vec3 ambient_material;
    Vec3 diffuse_material;
    Vec3 specular_material;
    double shininess;

    float index_medium;
    float transparency;

    MaterialType type;

    Material()
    {
        type = Material_Diffuse_Blinn_Phong;
        transparency = 0.0;
        index_medium = 1.0;
        ambient_material = Vec3(0., 0., 0.);
    }

    bool has_diffuse_tex = false;
    bool has_normal_map = false;

    ppmLoader::ImageRGB diffuse_tex;
    ppmLoader::ImageRGB normal_map; // normal map (tangent-space)

    bool is_emissive = false;
    Vec3 emission_color = Vec3(0.f, 0.f, 0.f);
    float emission_strength = 0.f;
};

#endif // MATERIAL_H
