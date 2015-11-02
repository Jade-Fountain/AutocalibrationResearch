
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

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/matrix_inverse.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/ext.hpp"

#ifdef __APPLE__
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
#include "SensorPlant.h"
#include "MocapStream.h"


autocal::TimeStamp psMoveLatency = -1400;
bool paused = false;
//Define the key input callback  
static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)  
{  
    if(action == GLFW_PRESS){
        if (key == GLFW_KEY_ESCAPE) {
            glfwSetWindowShouldClose(window, GL_TRUE);  
        } else if(key == GLFW_KEY_COMMA) {
            psMoveLatency = psMoveLatency - 1000;
            std::cout << "psMoveLatency = " << psMoveLatency << std::endl;
        } else if(key == GLFW_KEY_PERIOD){
            psMoveLatency = psMoveLatency + 1000;
            std::cout << "psMoveLatency = " << psMoveLatency << std::endl;
        } else if(key == GLFW_KEY_P){
            paused = !paused;
        }
    }
        
} 

int main(int argc, char* argv[])  
{  
    std::string video_filename;
    if(argc > 1){
        video_filename = argv[1];
    }
    std::cout << "Loading video file " << video_filename << std::endl;

    CvCapture* video = cvCaptureFromFile(video_filename.c_str());
    int fps, width, height;
    if(video == NULL){
        std::cout << "Video load failed... Exiting" << std::endl;
        return -1;
    } else {
        fps = ( int )cvGetCaptureProperty( video, CV_CAP_PROP_FPS );
        // fps = 30; 
        width = ( int )cvGetCaptureProperty( video, CV_CAP_PROP_FRAME_WIDTH ); 
        height = ( int )cvGetCaptureProperty( video, CV_CAP_PROP_FRAME_HEIGHT ); 
        std::cout << "Video load successful... FPS = " << fps << std::endl;
    }
    float frame_duration = 1.0 / float(fps);
    

    //Declare a window object  
    GLFWwindow* window = setUpGLWindow(width, height);
    //Sets the key callback  
    glfwSetKeyCallback(window, key_callback);  
  
    //Some GL options to configure for drawing
    glEnable(GL_TEXTURE_2D);
    GLuint texture;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);


    //Mocap stream stuff
    //Start timestamp
    std::string timeString = video_filename.substr(0,video_filename.size() - 4); //Remove .avi from filename
    autocal::TimeStamp videoStartTime = std::stoll(timeString);
    std::cout << "videoStartTime = " << videoStartTime << " micro sec" << std::endl;
    //Mocap stream
    autocal::MocapStream psmoveStream("psmove", false);
    psmoveStream.loadMocapData("psmovedata", videoStartTime, std::chrono::system_clock::now());

    autocal::MocapStream optitrackStream("mocap", true);
    bool optitrackReflectZ = true;
    std::set<int> mocapAllowedIDs = {1,2};
    optitrackStream.loadMocapData("mocapdata", videoStartTime,std::chrono::system_clock::now(), optitrackReflectZ, mocapAllowedIDs);

    bool useSimulation = false;
    autocal::SensorPlant sensorPlant(useSimulation);
    if(useSimulation){
        //Exp 4 -...
        autocal::MocapStream::SimulationParameters a1; 
        autocal::MocapStream::SimulationParameters a2;
        autocal::MocapStream::SimulationParameters d1; 
        autocal::MocapStream::SimulationParameters d2; 
        a2.noise.angle_stddev = 1;
        d1.noise.disp_stddev = 2;
        d2.noise.disp_stddev = 10;
        int aN = 10;
        int dN = 10;
        sensorPlant.setSimParameters(a1,a2,aN,d1,d2,dN);
    }
    sensorPlant.addStream(psmoveStream);
    sensorPlant.addStream(optitrackStream);

    //Ground Truth
    Transform3D psmoveToMocap;

    arma::vec3 right = { 1.198981,  0.129528, 0.200404};
    arma::vec3 left =  { 1.202117,  0.133606,  0.297056};
    arma::vec3 centre1 =  { 1.164790,  0.0117017, 0.267929};
    arma::vec3 centre2 =   { 1.162927,  0.116876,  0.230719};
    arma::vec3 viewPoint =  {-1.835153,  1.196957,  0.274641};
    
    arma::vec3 centre = 0.5*(centre2 + centre1);
    
    psmoveToMocap.translation() = centre;
    psmoveToMocap.z() = -arma::normalise(viewPoint - centre);
    
    arma::vec3 psuedoX =  arma::normalise(left - right);
    psmoveToMocap.y() = arma::normalise(arma::cross(psuedoX,psmoveToMocap.z()));
    psmoveToMocap.x() = arma::cross(psmoveToMocap.z(), psmoveToMocap.y());

//--------------------------------------------------------------------
    // Ground truth exp 1
    // arma::vec3 centre =     {   1.0205,    0.6637,  0.025500};
    // arma::vec3 frontRight = { 1.007544,  0.662266, -0.141178};
    // arma::vec3 frontLeft =  { 1.016013,  0.661523,  0.196680};
    // arma::vec3 backRight =  { 1.024875,  0.659862, -0.144267};
    // arma::vec3 backLeft =   { 1.032640,  0.658253,  0.191176};
    // // arma::vec3 top =        { 1.022050,  0.677163,  0.026080};
    // arma::vec3 viewPoint =  {-1.092194,  0.977327,  0.076759};

    // psmoveToMocap.translation() = centre;
    // psmoveToMocap.z() = -arma::normalise(viewPoint - centre);

    // arma::vec3 psuedoX =  - (frontRight + backRight) / 2 + (frontLeft + backLeft) / 2;
    // psmoveToMocap.y() = arma::normalise(arma::cross(psuedoX,psmoveToMocap.z()));
    // psmoveToMocap.x() = arma::cross(psmoveToMocap.z(), psmoveToMocap.y());
//--------------------------------------------------------------------

    std::cout << "psmoveToMocap = \n" << psmoveToMocap << std::endl;

    bool useTruthForMatching = false;
    sensorPlant.setGroundTruthTransform("mocap","psmove", psmoveToMocap.i(), useTruthForMatching);

    //Main Loop
    auto start = std::chrono::steady_clock::now();  
    int video_frames = 0; 
    do  
    {  
        //Get and organize events, like keyboard and mouse input, window resizing, etc...  
        glfwPollEvents(); 

        // frames++;
        auto now = std::chrono::steady_clock::now();    
        double frame_time_since_start = std::chrono::duration_cast<std::chrono::milliseconds>(now-start).count() / float(std::milli::den);  

        autocal::TimeStamp current_timestamp = videoStartTime + std::chrono::duration_cast<std::chrono::microseconds>(now-start).count();
        
        if(video_frames * frame_duration < frame_time_since_start && !paused){
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
        
        //setup overgraphics
        glClear(GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_PROJECTION);
        glm::mat4 proj =

            // glm::perspectiveFov<float>(PSEYE_FOV_BLUE_DOT,
            // image->width, image->height, 0.01f, 10.0f);

         glm::perspective(  float(PSEYE_FOV_BLUE_DOT * 3.14159 / 180.0),            //VERTICAL FOV
                                            float(image->width) / float(image->height),  //aspect ratio
                                            0.01f,         //near plane distance (min z)
                                            10.0f           //Far plane distance (max z)
                                            );
        glLoadMatrixf(glm::value_ptr(proj));

        std::vector<std::pair<int,int>> matches = sensorPlant.matchStreams("psmove","mocap",current_timestamp, psMoveLatency);

        autocal::MocapStream::Frame psmoveFrame = sensorPlant.getStream("psmove").getFrame(current_timestamp + psMoveLatency);
        // autocal::MocapStream::Frame psmoveFrame = psmoveStream.getFrame(current_timestamp + psMoveLatency);
        Transform3D psmovePose;
        for(auto& pair : psmoveFrame.rigidBodies){
            auto& rigidBodyID = pair.first;
            auto& rigidBody = pair.second;
            psmovePose = rigidBody.pose;
            // std::cout << "psmove pose = \n" << psmovePose << std::endl;
            glMatrixMode(GL_MODELVIEW);
            glLoadMatrixd(psmovePose.memptr());  
            
            drawBasis(0.1);
        } 

        autocal::MocapStream::Frame optitrackFrame = sensorPlant.getGroundTruth("mocap", "psmove", current_timestamp);
        // autocal::MocapStream::Frame optitrackFrame = sensorPlant.getStream("mocap").getFrame(current_timestamp);
        for(auto& pair : optitrackFrame.rigidBodies){
            auto& rigidBodyID = pair.first;
            auto& rigidBody = pair.second;
            Transform3D pose = rigidBody.pose;
            // std::cout << "mocap RB" << rigidBodyID << " pose = \n" << pose << std::endl;
            // Transform3D pose = psmoveToMocap.i() * rigidBody.pose;
            // pose = pose.i();
            glMatrixMode(GL_MODELVIEW);
            glLoadMatrixd(pose.memptr());  
            if(matches.size() > 0 && matches[0].second == rigidBodyID) {
                glEnable(GL_LIGHTING);
                GLfloat z_diff[4] = {1.0, 1.0, 1.0, 1.0};
                glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, z_diff);
                glutSolidSphere(0.05, 10, 10);
                // std::cout << "matches RB" << rigidBodyID << std::endl; 
            }
            drawBasis(0.1);
        } 


        //Swap buffers  
        glfwSwapBuffers(window);  
 
  
    } while (!glfwWindowShouldClose(window));   //Check if the ESC key had been pressed or if the window had been closed
    auto now = std::chrono::steady_clock::now();    
    double finish_time = std::chrono::duration_cast<std::chrono::milliseconds>(now-start).count() / float(std::milli::den);     
    // std::cout << "average draw framerate = " << double(frames) / finish_time << " Hz " << std::endl; 
    std::cout << "average video framerate = " << double(video_frames) / finish_time << " Hz " << std::endl; 
    sensorPlant.next();
    cvReleaseCapture(&video);  
    destroyGLWindow(window);
}  