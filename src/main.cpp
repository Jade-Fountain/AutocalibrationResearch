
 /**
 * modified from:
 * PS Move API - An interface for the PS Move Motion Controller
 * Copyright (c) 2012 Thomas Perl <m@thp.io>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 **/

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

Tracker::Tracker()
    : m_moves(NULL),
      m_count(psmove_count_connected()),
      m_tracker(psmove_tracker_new()),
      m_fusion(psmove_fusion_new(m_tracker, 1., 1000.))
{
    // PSMove *move;
    // move = psmove_connect();
    // if (move == NULL) {
    //     printf("Could not connect to default Move controller.\n"
    //            "Please connect one via USB or Bluetooth.\n");
    //     exit(1);
    // }
    // char *serial = psmove_get_serial(move);
    // auto ctype = psmove_connection_type(move);
    // switch (ctype) {
    //     case Conn_USB:
    //         printf("Connected via USB.\n");
    //         break;
    //     case Conn_Bluetooth:
    //         printf("Connected via Bluetooth.\n");
    //         break;
    //     case Conn_Unknown:
    //         printf("Unknown connection type.\n");
    //         break;
    // }


    psmove_tracker_set_mirror(m_tracker, PSMove_True);
    psmove_tracker_set_exposure(m_tracker, Exposure_HIGH);

    m_moves = (PSMove**)calloc(m_count, sizeof(PSMove*));
    m_items = (int*)calloc(m_count, sizeof(int));
    for (int i=0; i<m_count; i++) {
        m_moves[i] = psmove_connect_by_id(i);
        m_items[i] = WIRE_CUBE;

        psmove_enable_orientation(m_moves[i], PSMove_True);
        assert(psmove_has_orientation(m_moves[i]));
        
        while (psmove_tracker_enable(m_tracker, m_moves[i]) != Tracker_CALIBRATED);
    }
}

Tracker::~Tracker()
{
    psmove_fusion_free(m_fusion);
    psmove_tracker_free(m_tracker);
    for (int i=0; i<m_count; i++) {
        psmove_disconnect(m_moves[i]);
    }
    free(m_items);
    free(m_moves);
}

void
Tracker::update()
{
    for (int i=0; i<m_count; i++) {
        while (psmove_poll(m_moves[i]));        

        float x, y, z;
        psmove_fusion_get_position(m_fusion, m_moves[i],
                &x, &y, &z);

        int buttons = psmove_get_buttons(m_moves[i]);
        if (buttons & Btn_MOVE) {
            psmove_reset_orientation(m_moves[i]);
        } else if (buttons & Btn_PS) {
            exit(0);
        } else if (buttons & Btn_SELECT) {
            m_rotation += 2.;
        } else if (buttons & Btn_CROSS) {
            m_trace.push_back(Point3D(x, y, z));
        }

        if (buttons & Btn_START) {
            if (m_has_last_offset) {
                m_offset = Point3D(m_offset.x + x - m_last_offset.x,
                        m_offset.y + y - m_last_offset.y,
                        m_offset.z + z - m_last_offset.z);
            } else {
                m_has_last_offset = true;
            }
            m_last_offset = Point3D(x, y, z);
        } else {
            m_has_last_offset = false;
        }

        unsigned int pressed, released;
        psmove_get_button_events(m_moves[i], &pressed, &released);
        if (pressed & Btn_SQUARE) {
            m_items[i] -= 1;
            if (m_items[i] < 0) m_items[i] = ITEM_MAX - 1;
        } else if (pressed & Btn_TRIANGLE) {
            m_items[i] += 1;
            if (m_items[i] == ITEM_MAX) m_items[i] = 0;
        } else if (pressed & Btn_CIRCLE) {
            m_trace.clear();
            m_rotation = 0.;
            m_offset = Point3D(0., 0., 0.);
        }
    }

    psmove_tracker_update_image(m_tracker);
    psmove_tracker_update(m_tracker, NULL);
}

void
Tracker::init()
{
    glEnable(GL_TEXTURE_2D);
    glGenTextures(1, &m_texture);
    glBindTexture(GL_TEXTURE_2D, m_texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
}

void
Tracker::render()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    PSMoveTrackerRGBImage image = psmove_tracker_get_image(m_tracker);

    glEnable(GL_TEXTURE_2D);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image.width, image.height,
            0, GL_RGB, GL_UNSIGNED_BYTE, image.data);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    /* Draw the camera image, filling the screen */
    glColor3f(1., 1., 1.);
    glBegin(GL_QUADS);
    glTexCoord2f(0., 1.);
    glVertex2f(-1., -1.);
    glTexCoord2f(1., 1.);
    glVertex2f(1., -1.);
    glTexCoord2f(1., 0.);
    glVertex2f(1., 1.);
    glTexCoord2f(0., 0.);
    glVertex2f(-1., 1.);
    glEnd();

    glDisable(GL_TEXTURE_2D);

    /* Clear the depth buffer to allow overdraw */
    glClear(GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(psmove_fusion_get_projection_matrix(m_fusion));

    /* Render the trace */
    if (m_trace.size()) {
        Point3D center = m_trace[0];
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glTranslatef(center.x + m_offset.x, center.y + m_offset.y, center.z + m_offset.z);
        glRotatef(m_rotation, 0., 1., 0.);

        std::vector<Point3D>::iterator it;
        glColor3f(1., 0., 0.);
        glEnable(GL_LIGHTING);
        //glBegin(GL_TRIANGLE_STRIP);
        for (it=m_trace.begin(); it != m_trace.end(); ++it) {
            Point3D point = *it;
            Point3D moved(point.x - center.x,
                    point.y - center.y,
                    point.z - center.z);
            //glVertex3f(moved.x, moved.y, moved.z);
            glPushMatrix();
            glTranslatef(moved.x, moved.y, moved.z);
            glutSolidCube(.5);
            glPopMatrix();
        }
        //glEnd();
        glDisable(GL_LIGHTING);
    }

    for (int i=0; i<m_count; i++) {
        glMatrixMode(GL_MODELVIEW);
        glLoadMatrixf(psmove_fusion_get_modelview_matrix(m_fusion, m_moves[i]));

        if (m_items[i] == WIRE_CUBE) {
            glColor3f(1., 0., 0.);
            glutWireCube(1.);
            glColor3f(0., 1., 0.);

            glPushMatrix();
            glScalef(1., 1., 4.5);
            glTranslatef(0., 0., -.5);
            glutWireCube(1.);
            glPopMatrix();

            glColor3f(0., 0., 1.);
            glutWireCube(3.);
        } else if (m_items[i] == SOLID_CUBE) {
            glEnable(GL_LIGHTING);
            glutSolidCube(2.);
            glDisable(GL_LIGHTING);
        } else if (m_items[i] == SOLID_TEAPOT) {
            glEnable(GL_LIGHTING);
            glPushMatrix();
            glRotatef(90., 1., 0., 0.);
            glutSolidTeapot(1.);
            glPopMatrix();
            glDisable(GL_LIGHTING);
        }
    }
}

//Define an error callback  
static void error_callback(int error, const char* description)  
{  
    fputs(description, stderr);  
    getchar();  
}  
  
//Define the key input callback  
static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)  
{  
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)  
    glfwSetWindowShouldClose(window, GL_TRUE);  
   
} 

static void cursor_position_callback(GLFWwindow* window, double xpos, double ypos)
{
}

static void handle_input(GLFWwindow* window, double time_since_start){

}

static void checkGLError(){

    GLenum error;
    while((error = glGetError()) != GL_NO_ERROR){  
        std::cout << "GL Error = " << std::endl;  
        switch(error)
        {
            case 0:
                std::cout <<"Internal error in glGetError()" << std::endl;
                break;
            
            case GL_INVALID_ENUM:
                std::cout <<"Invalid enum" << std::endl;
                break;
            
            case GL_INVALID_VALUE:
                std::cout <<"Invalid value" << std::endl;
                break;
            
            case GL_INVALID_OPERATION:
                std::cout <<"Invalid operation" << std::endl;
                break;
            
            case GL_STACK_OVERFLOW:
                std::cout <<"Stack overflow" << std::endl;
                break;
            
            case GL_STACK_UNDERFLOW:
                std::cout <<"Stack underflow" << std::endl;
                break;
            
            case GL_OUT_OF_MEMORY:
                std::cout <<"Out of memory" << std::endl;
                break;
            
            case GL_TABLE_TOO_LARGE:
                std::cout <<"Table too large" << std::endl;
                break;
            
            default:
                std::cout << "Unknown error "<< error << std::endl;
        }
    }
    
}

int main( void )  
{  
    //Set the error callback  
    glfwSetErrorCallback(error_callback);  
  
    //Initialize GLFW  
    if (!glfwInit())  
    {  
        exit(EXIT_FAILURE);  
    }  
  
    //Set the GLFW window creation hints - these are optional  
    //glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3); //Request a specific OpenGL version  
    //glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2); //Request a specific OpenGL version  
    //glfwWindowHint(GLFW_SAMPLES, 4); //Request 4x antialiasing  
    //glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  
  
    //Declare a window object  
    GLFWwindow* window;  
  
    //Create a window and create its OpenGL context  
    window = glfwCreateWindow(640, 480, "Test Window", NULL, NULL);  
  
    //If the window couldn't be created  
    if (!window)  
    {  
        fprintf( stderr, "Failed to open GLFW window.\n" );  
        glfwTerminate();  
        exit(EXIT_FAILURE);  
    }  
  
    //This function makes the context of the specified window current on the calling thread.   
    glfwMakeContextCurrent(window);  
  
    //Sets the key callback  
    glfwSetKeyCallback(window, key_callback);  
  
    //Initialize GLEW  
    glewExperimental = GL_TRUE;
    GLenum err = glewInit();  
  
    //If GLEW hasn't initialized  
    if (err != GLEW_OK)   
    {  
        fprintf(stderr, "Error: %s\n", glewGetErrorString(err));  
        return -1;  
    }

    //TODO: create Vertex array object

    //TODO: load vertices and bind vertex buffer
    
    //TODO: create and bind element buffer

    Tracker psmoveTracker;
    psmoveTracker.init();

    //Set a background color  
    glClearColor(0.0f, 0.0f, 1.0f, 0.0f);  
    glEnable(GL_LIGHT0);
    glEnable(GL_DEPTH_TEST);
  
    //Main Loop  
    do  
    {  
        //Clear color buffer  
        glClear(GL_COLOR_BUFFER_BIT); 
        
        //TODO: Draw the graphics
        psmoveTracker.update();
        psmoveTracker.render();

        
        //Swap buffers  
        glfwSwapBuffers(window);  
        //Get and organize events, like keyboard and mouse input, window resizing, etc...  
        glfwPollEvents();  
  
    } //Check if the ESC key had been pressed or if the window had been closed  
    while (!glfwWindowShouldClose(window));  
  
    //Close OpenGL window and terminate GLFW  
    glfwDestroyWindow(window);  
    //Finalize and clean up GLFW  
    glfwTerminate();  
  
    exit(EXIT_SUCCESS);  
}  