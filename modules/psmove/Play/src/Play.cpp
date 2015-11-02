/*
 * This file is part of Autocalibration Codebase.
 *
 * The Autocalibration Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The Autocalibration Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the Autocalibration Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2015 Autocalibration <nubots@nubots.net>
 */

#include "Play.h"

#include "messages/support/Configuration.h"

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

#include "utility/autocal/PSMoveUtils.h"
#include "utility/autocal/GraphicsTools.h"

namespace modules {
namespace psmove {

	using autocal::SensorPlant;
	using autocal::MocapStream;

    using messages::support::Configuration;

    //Define the key input callback  
	void Play::handleInput(const sf::Window& w, double time_since_start){
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Escape))
		{
		    std::cout << "Exiting .." << std::endl;
		    running = false;
		}
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::Comma)){
            psMoveLatency = psMoveLatency - 1000;
            std::cout << "psMoveLatency = " << psMoveLatency << std::endl;
        }
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::Period)){
            psMoveLatency = psMoveLatency + 1000;
            std::cout << "psMoveLatency = " << psMoveLatency << std::endl;
        }
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::P)){
        	paused = !paused;
        }
		// sf::Event event;
  //       while (window.pollEvent(event))
  //       {
  //           if (event.type == sf::Event::Closed)
  //           {
  //               // end the program
  //               running = false;
  //           }
  //           else if (event.type == sf::Event::Resized)
  //           {
  //               // adjust the viewport when the window is resized
  //               glViewport(0, 0, event.size.width, event.size.height);
  //           }
  //       }
	}

    Play::Play(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)),window(sf::VideoMode(640*2, 480*2), "OpenGL", sf::Style::Default, sf::ContextSettings(32)) {

        on<Configuration>("Play.yaml").then([this] (const Configuration& config) {
        	psMoveLatency = config["psmove_start_latency"].as<long long int>();
            // Use configuration here from file Play.yaml
            // log("loading config: ", config["test"].as<std::string>());
			video_filename = config["video_filename"].as<std::string>();

        });

        on<Startup>().then([this]{

	        window.setActive(true);

		    std::cout << "Loading video file " << video_filename << std::endl;

		    video = cvCaptureFromFile(video_filename.c_str());
		    int fps, width, height;
		    if(video == NULL){
		        std::cout << "Video load failed... Exiting" << std::endl;
		        return -1;
		    } else {
		        fps = ( int )cvGetCaptureProperty( video, CV_CAP_PROP_FPS );
		        // fps = 21; 
		        width = ( int )cvGetCaptureProperty( video, CV_CAP_PROP_FRAME_WIDTH ); 
		        height = ( int )cvGetCaptureProperty( video, CV_CAP_PROP_FRAME_HEIGHT ); 
		        std::cout << "Video load successful... FPS = " << fps << std::endl;
		    }
		    frame_duration = 1.0 / float(fps);
		  
   		    bool success = setUpOpenGL();
	  	    if(!success){
	  	    	std::cout << "OpenGL Setup Failed! Shutting down" << std::endl; 
			    powerplant.shutdown();
			}
			
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
		    videoStartTime = std::stoll(timeString);
		    std::cout << "videoStartTime = " << videoStartTime << " micro sec" << std::endl;
		    //Mocap stream
		    autocal::MocapStream psmoveStream("psmove", false);
		    psmoveStream.loadMocapData("psmovedata", videoStartTime, std::chrono::system_clock::now());

		    autocal::MocapStream optitrackStream("mocap", true);
		    bool optitrackReflectZ = true;
		    std::set<int> mocapAllowedIDs = {1,2};
		    optitrackStream.loadMocapData("mocapdata", videoStartTime,std::chrono::system_clock::now(), optitrackReflectZ, mocapAllowedIDs);

		    bool useSimulation = false;
		    sensorPlant = SensorPlant(useSimulation);
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
		    start = std::chrono::steady_clock::now();  
		    video_frames = 0; 

        });

        on<Every<60,Per<std::chrono::seconds>>, Single>().then([this]{
	       	// std::cout << "Frame " << video_frames << std::endl; 

	        auto now = std::chrono::steady_clock::now();    
	        double frame_time_since_start = std::chrono::duration_cast<std::chrono::milliseconds>(now-start).count() / float(std::milli::den);  

	        autocal::TimeStamp current_timestamp = videoStartTime + std::chrono::duration_cast<std::chrono::microseconds>(now-start).count();
	        
	        window.setActive(true);
	        handleInput(window, frame_time_since_start);

	        if(video_frames * frame_duration < frame_time_since_start && !paused){
	            video_frames++;
	        } else {
	            return;
	        }


	        //Clear color buffer  
	        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	        
	        glEnable(GL_TEXTURE_2D);

	        IplImage* image = cvQueryFrame(video);
	        if(image == NULL){
	            std::cout << "no images left in video file" << std::endl;
	            powerplant.shutdown();
	            return;
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

	        if(sensorPlant.streamNotEmpty("mocap")){
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
		                GLfloat diff[4] = {1.0, 1.0, 1.0, 1.0};
		                glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
		                glutSolidSphere(0.05, 10, 10);
		                // std::cout << "matches RB" << rigidBodyID << std::endl; 
		            }
		            drawBasis(0.1);
		        } 
			}


	        window.display();

		    if(!running){
		    	powerplant.shutdown();
		    }

        });

		on<Shutdown>().then([this]{
			auto now = std::chrono::steady_clock::now();    
		    double finish_time = std::chrono::duration_cast<std::chrono::milliseconds>(now-start).count() / float(std::milli::den);     
		    // std::cout << "average draw framerate = " << double(frames) / finish_time << " Hz " << std::endl; 
		    std::cout << "average video framerate = " << double(video_frames) / finish_time << " Hz " << std::endl; 
		    sensorPlant.next();
		    cvReleaseCapture(&video);  
		});
    }
}
}
