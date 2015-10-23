
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

#include <iostream>

#include <vector>

#include <chrono>
#include <iostream>

#ifdef __APPLE__
    // #include <OpenGL/gl.h>
    #include <GL/glew.h>  

    #include <GLFW/glfw3.h>  
    #include <GLUT/glut.h>
#else
  #include <GL/gl.h>
  #include <GL/glut.h>
#endif 

#include "opencv2/core/core_c.h"
#include "opencv2/highgui/highgui_c.h"


#include "PSMoveUtils.h"
#include "GraphicsTools.h"

int main( void )  
{  
    int width = 640;
    int height = 480;

    //Declare a window object  
    GLFWwindow* window = setUpGLWindow(width, height);

    Tracker psmoveTracker;

    psmoveTracker.init();
    
    std::stringstream filename;
    auto start_time = std::chrono::system_clock::now();
    filename << std::to_string(std::chrono::duration_cast<std::chrono::microseconds>(start_time.time_since_epoch()).count()) << ".avi";
    int video_frames = 0; 


    //TODO: use opencv c++ bindings
    CvVideoWriter *writer = cvCreateVideoWriter(filename.str().c_str(),
        CV_FOURCC('M','J','P','G'), 30, cvSize(width, height), 1);
    //Main Loop  
    do  
    {   
        video_frames++;
        auto now = std::chrono::system_clock::now();    
        double frame_time_since_start = std::chrono::duration_cast<std::chrono::milliseconds>(now-start_time).count() / float(std::milli::den);  
        std::cout << "Frame time = " << frame_time_since_start << std::endl;

        //Clear color buffer  
        glClear(GL_COLOR_BUFFER_BIT); 
        
        psmoveTracker.update();
        psmoveTracker.render();
        psmoveTracker.saveFrame(writer);
        psmoveTracker.savePoses();

        drawCrossHair();

        //Swap buffers  
        glfwSwapBuffers(window);  
        //Get and organize events, like keyboard and mouse input, window resizing, etc...  
        glfwPollEvents();  
  
    } //Check if the ESC key had been pressed or if the window had been closed  
    while (!glfwWindowShouldClose(window));  
    auto now = std::chrono::system_clock::now();    
    double finish_time = std::chrono::duration_cast<std::chrono::milliseconds>(now-start_time).count() / float(std::milli::den);     
    // std::cout << "average draw framerate = " << double(frames) / finish_time << " Hz " << std::endl; 
    std::cout << "average video framerate = " << double(video_frames) / finish_time << " Hz " << std::endl; 
    destroyGLWindow(window);
}  