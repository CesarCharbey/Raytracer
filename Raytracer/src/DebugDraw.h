#pragma once
#include "Vec3.h"
#include <GL/glut.h>

// Dessine une AABB en fil de fer (LIGNES)
inline void drawAABBWire(const Vec3 &bmin, const Vec3 &bmax)
{
    const float x0 = bmin[0], y0 = bmin[1], z0 = bmin[2];
    const float x1 = bmax[0], y1 = bmax[1], z1 = bmax[2];

    glBegin(GL_LINES);
    // bas
    glVertex3f(x0, y0, z0);
    glVertex3f(x1, y0, z0);
    glVertex3f(x1, y0, z0);
    glVertex3f(x1, y0, z1);
    glVertex3f(x1, y0, z1);
    glVertex3f(x0, y0, z1);
    glVertex3f(x0, y0, z1);
    glVertex3f(x0, y0, z0);
    // haut
    glVertex3f(x0, y1, z0);
    glVertex3f(x1, y1, z0);
    glVertex3f(x1, y1, z0);
    glVertex3f(x1, y1, z1);
    glVertex3f(x1, y1, z1);
    glVertex3f(x0, y1, z1);
    glVertex3f(x0, y1, z1);
    glVertex3f(x0, y1, z0);
    // montants
    glVertex3f(x0, y0, z0);
    glVertex3f(x0, y1, z0);
    glVertex3f(x1, y0, z0);
    glVertex3f(x1, y1, z0);
    glVertex3f(x1, y0, z1);
    glVertex3f(x1, y1, z1);
    glVertex3f(x0, y0, z1);
    glVertex3f(x0, y1, z1);
    glEnd();
}
