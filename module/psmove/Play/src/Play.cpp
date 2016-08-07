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

#include "message/support/Configuration.h"

#include <stdio.h>
#include <iostream>

#include <time.h>
#include <unistd.h>
#include <assert.h>
#include <math.h>

#include <vector>

#include <chrono>

#include "utility/autocal/PSMoveUtils.h"
#include "utility/autocal/GraphicsTools.h"

namespace module {
namespace psmove {

	using autocal::SensorPlant;
	using autocal::MocapStream;

    using message::support::Configuration;

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

		    use_simulation = config["use_simulation"].as<bool>();

        });

        on<Startup>().then("Startup",[this]{
        	//Set window active
	        window.setActive(true);

	        //Load video file and paramters
		    std::cout << "Loading video file " << video_filename << std::endl;
		    video = cvCaptureFromFile(video_filename.c_str());
		    int fps, width, height;
		    if(video == NULL){
		        std::cout << "Video load failed... Exiting" << std::endl;
		        // return -1;
		    } else {
		        fps = ( int )cvGetCaptureProperty( video, CV_CAP_PROP_FPS );
		        // fps = 21; 
		        width = ( int )cvGetCaptureProperty( video, CV_CAP_PROP_FRAME_WIDTH ); 
		        height = ( int )cvGetCaptureProperty( video, CV_CAP_PROP_FRAME_HEIGHT ); 
		        std::cout << "Video load successful... FPS = " << fps << std::endl;
		    }
		    frame_duration = 1.0 / float(fps);
		    //Start timestamp
		    std::string timeString = video_filename.substr(0,video_filename.size() - 4); //Remove .avi from filename
		    videoStartTime = std::stoll(timeString);
		  
		  	//Init opengl
   		    bool success = setUpOpenGL();
	  	    if(!success){
	  	    	std::cout << "OpenGL Setup Failed! Shutting down" << std::endl; 
			    powerplant.shutdown();
			}
			
		    //Some GL options particular to this role
		    //enables textures
		    glEnable(GL_TEXTURE_2D);
		    GLuint texture;
		    glGenTextures(1, &texture);
		    glBindTexture(GL_TEXTURE_2D, texture);
		    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

		    //PSmove pose stream
		    autocal::MocapStream psmoveStream("psmove", true);

		    //Mocap pose stream
		    autocal::MocapStream optitrackStream("mocap", false, true);
		    //coordinate system is LH while psmove is RH
		    std::set<int> mocapAllowedIDs = {1,2};
		    optitrackStream.loadMocapData("mocapdata", videoStartTime,std::chrono::system_clock::now(), mocapAllowedIDs);

		    //Initialise sensor plant
		    sensorPlant = SensorPlant();
		    //Optional simulation parameters
		    if(use_simulation){
		        //Exp 4 -...
		        autocal::SimulationParameters a1; 
		        autocal::SimulationParameters a2;
		        autocal::SimulationParameters d1; 
		        autocal::SimulationParameters d2; 
		        a1.noise.angle_stddev = 0;
		        // a2.noise.angle_stddev = 1;
		        d1.noise.disp_stddev = 0;
		        // d2.noise.disp_stddev = 10;
		        int aN = 1;
		        int dN = 1;
		        sensorPlant.setSimParameters(a1,a2,aN,d1,d2,dN);
		        //set the simulated connections between rigid bodies
		        std::map<int,int> answers;
		        answers[1] = 1;
		        // answers[2] = 2;
		        sensorPlant.setAnswers("psmove","mocap",answers);
		        psmoveStream.setupSimulation(optitrackStream, answers);

		    } else {
		    	psmoveStream.loadMocapData("psmovedata", videoStartTime, std::chrono::system_clock::now());
		        std::map<int,int> answers;
		        answers[0] = 2;
		        sensorPlant.setAnswers("psmove","mocap",answers);
		    }

		    //Push back the loaded streams
		    //Ensure simulation is setup for sensor plant prior to adding
		    sensorPlant.addStream(psmoveStream);
		    sensorPlant.addStream(optitrackStream);

		    //Ground Truth computation
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
		    psmoveToMocap.x() = -arma::cross(psmoveToMocap.z(), psmoveToMocap.y());

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

		    //Ground truth pass through to sensor plant
		    bool useTruthForMatching = false;
		    sensorPlant.setGroundTruthTransform("mocap","psmove", psmoveToMocap.i(), useTruthForMatching);

		    //Record start parameters
		    start = std::chrono::steady_clock::now();  
		    video_frames = 0; 

        });

        on<Every<60,Per<std::chrono::seconds>>, Single>().then("Main Loop",[this]{
        	//Get current time
	        auto now = std::chrono::steady_clock::now();    
	        double frame_time_since_start = std::chrono::duration_cast<std::chrono::milliseconds>(now-start).count() / float(std::milli::den);  
	        
	        //Compute timestamp in microseconds
	        autocal::TimeStamp current_timestamp = videoStartTime + std::chrono::duration_cast<std::chrono::microseconds>(now-start).count();
	        
	        //Set window active
	        window.setActive(true);

	        //Check input
	        handleInput(window, frame_time_since_start);

	        //Check if new frame is ready to be drawn
	        if(video_frames * frame_duration < frame_time_since_start && !paused){
	            video_frames++;
	        } else {
	            return;
	        }

	        //Clear color buffer and enable textures
	        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	        glEnable(GL_TEXTURE_2D);

	        //Load next image frame
	        if(video != NULL){
	        	running = drawCamera(video, PSEYE_FOV_BLUE_DOT) && running;
	        }

	        //Match the streams
	        std::vector<std::pair<int,int>> matches = sensorPlant.matchStreams("psmove","mocap",current_timestamp, psMoveLatency);

	        
	        //Draw psmove
	        drawSensorStreams(sensorPlant,
	        				  "psmove", //reference frame
	        				  "mocap", //matching range (ie stream to match to, usually the same as reference frame, but not this time)
	        				  current_timestamp,
	        				  matches);

	        window.display();

	        

		    if(!running){
			    //Load next sim params, or end if there are none
		    	bool next = sensorPlant.next();
		    	
		    	if(next) {
		    		// reset();
		    	} else {

		    	}
		    	powerplant.shutdown();

		    }

        });

        on<Always>().then("event handler",[this]{
			// check all the window's events that were triggered since the last iteration of the loop
	        sf::Event event;
	        while (window.pollEvent(event))
	        {
	            // "close requested" event: we close the window
	            if (event.type == sf::Event::Closed)
	                powerplant.shutdown();
	        }
        });

		on<Shutdown>().then("Shutdown",[this]{
			//Display draw framerate of the video
			auto now = std::chrono::steady_clock::now();    
		    double finish_time = std::chrono::duration_cast<std::chrono::milliseconds>(now-start).count() / float(std::milli::den);     
		    std::cout << "average video framerate = " << double(video_frames) / finish_time << " Hz " << std::endl; 
		    window.close();
		    //Release the video
		    cvReleaseCapture(&video);  
		});
    }
}
}
