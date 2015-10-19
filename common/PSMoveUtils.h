
#include <stdio.h>
#include <iostream>

#include <time.h>
#include <unistd.h>
#include <assert.h>
#include <math.h>

#include <vector>

#include <chrono>


#ifdef __APPLE__
    // #include <OpenGL/gl.h>
    #include <GL/glew.h>  

    #include <GLFW/glfw3.h>  
    #include <GLUT/glut.h>
#else
  #include <GL/gl.h>
  #include <GL/glut.h>
#endif 

#include "psmoveapi/psmove.h"
#include "psmoveapi/psmove_tracker.h"
#include "psmoveapi/psmove_fusion.h"

#ifndef PSMOVE_UTILS_H
#define PSMOVE_UTILS_H

enum {
    NOTHING,
    WIRE_CUBE,
    SOLID_CUBE,
    SOLID_TEAPOT,
    ITEM_MAX,
};

class Point3D {
    public:
        Point3D(float x=0, float y=0, float z=0) : x(x), y(y), z(z) {}

        float x;
        float y;
        float z;
};

class Tracker {
    public:
        Tracker();
        ~Tracker();
        void update();

        void init();
        void render();

    private:
        PSMove **m_moves;
        int *m_items;
        int m_count;

        std::vector<Point3D> m_trace;
        float m_rotation;
        Point3D m_offset;
        Point3D m_last_offset;
        bool m_has_last_offset;

        PSMoveTracker *m_tracker;
        PSMoveFusion *m_fusion;
        GLuint m_texture;
};


#endif