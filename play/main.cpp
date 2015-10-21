
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

#include "opencv2/opencv.hpp"

#include "PSMoveUtils.h"
#include "GraphicsTools.h"

int main(int argc, char* argv[])  
{  
    std::string video_filename;
    if(argc > 1){
        video_filename = argv[1];
    }
    std::cout << "Loading video file " << video_filename << std::endl;

    int width = 640;
    int height = 480;

    //Declare a window object  
    GLFWwindow* window = setUpGLWindow(width, height);

    CvCapture* video = cvCaptureFromFile(video_filename.c_str());
    int fps;
    if(video == NULL){
        std::cout << "Video load failed... Exiting" << std::endl;
        return -1;
    } else {
        fps = ( int )cvGetCaptureProperty( video, CV_CAP_PROP_FPS ); 
        std::cout << "Video load successful... FPS = " << fps << std::endl;
    }
    float frame_duration = 1.0 / float(fps);

    glEnable(GL_TEXTURE_2D);
    GLuint texture;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    //Main Loop
    auto start = std::chrono::steady_clock::now();  
    int video_frames = 0; 

    do  
    {  
        // frames++;   
        auto now = std::chrono::steady_clock::now();    
        double frame_time = std::chrono::duration_cast<std::chrono::milliseconds>(now-start).count() / float(std::milli::den);  
            

        if(video_frames * frame_duration < frame_time){
            video_frames++;
        } else {
            continue;
        }

        //Clear color buffer  
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        glEnable(GL_TEXTURE_2D);

        IplImage* image = cvQueryFrame(video);
        if(image == NULL){
            std::cout << "no images left in video file" << std::endl;
            break;
        }
      
        GLenum format;
        switch(image->nChannels) {
            case 1:
                format = GL_LUMINANCE;
                break;
            case 2:
                format = GL_LUMINANCE_ALPHA;
                break;
            case 3:
                format = GL_BGR;
                break;
            default:
                break;
        }

        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image->width, image->height,
                0, format, GL_UNSIGNED_BYTE, image->imageData);

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

        //Swap buffers  
        glfwSwapBuffers(window);  
        //Get and organize events, like keyboard and mouse input, window resizing, etc...  
        glfwPollEvents();  
  
    } while (!glfwWindowShouldClose(window));   //Check if the ESC key had been pressed or if the window had been closed
    auto now = std::chrono::steady_clock::now();    
    double finish_time = std::chrono::duration_cast<std::chrono::milliseconds>(now-start).count() / float(std::milli::den);     
    // std::cout << "average draw framerate = " << double(frames) / finish_time << " Hz " << std::endl; 
    std::cout << "average video framerate = " << double(video_frames) / finish_time << " Hz " << std::endl; 
    cvReleaseCapture(&video);  
    destroyGLWindow(window);
}  