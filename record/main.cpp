
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

#include "PSMoveUtils.h"

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