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

#include "LiveMatchMocap.h"

#include <iostream>

#include <vector>

#include <chrono>
#include <iostream>


#include "opencv2/core/core_c.h"
#include "opencv2/highgui/highgui_c.h"


#include "utility/autocal/GraphicsTools.h"
#include "messages/support/Configuration.h"
#include "messages/input/MotionCapture.h"
#include "utility/math/geometry/UnitQuaternion.h"
#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/matrix/Transform3D.h"

namespace modules {
namespace psmove {

    using messages::support::Configuration;
    using utility::math::geometry::UnitQuaternion;
	using messages::input::MotionCapture;
    using utility::math::matrix::Rotation3D;
    using utility::math::matrix::Transform3D;

    //Define the key input callback  
	void LiveMatchMocap::handleInput(const sf::Window& w, double time_since_start){
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Escape))
		{
		    std::cout << "Exiting .." << std::endl;
		    running = false;
		}
	}

    LiveMatchMocap::LiveMatchMocap(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)), 
    window(sf::VideoMode(640*4, 480*4), "OpenGL", sf::Style::Default, sf::ContextSettings(32))
     {

        on<Configuration>("LiveMatchMocap.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file LiveMatchMocap.yaml
        	width = config["width"].as<int>();
		    height = config["height"].as<int>();
	    	fps = config["fps"].as<float>();
	    	frame_duration = 1.0 / fps;
	    	use_simulation = config["use_simulation"].as<bool>();
        });

        on<Startup>().then([this]{
        	//Initialise sensor plant
		    sensorPlant = autocal::SensorPlant(use_simulation);
		    //Optional simulation parameters
		    if(use_simulation){
		        //Exp 4 -...
		        autocal::MocapStream::SimulationParameters a1; 
		        autocal::MocapStream::SimulationParameters a2;
		        autocal::MocapStream::SimulationParameters d1; 
		        autocal::MocapStream::SimulationParameters d2; 
		        int aN = 1;
		        int dN = 1;
		        sensorPlant.setSimParameters(a1,a2,aN,d1,d2,dN);
		    }

	        window.setActive(true);


		    //GLEW
		    bool success = setUpOpenGL();
	  	    if(!success){
	  	    	std::cout << "OpenGL Setup Failed! Shutting down" << std::endl; 
			    powerplant.shutdown();
			}
			//Psmove
		    psmoveTracker.init();
		    
		    //Check errors
		    checkGLError();
	        
	        start_time = std::chrono::system_clock::now();    
  			start_timestamp = std::chrono::duration_cast<std::chrono::microseconds>(start_time.time_since_epoch()).count();
        });

		   //Main Loop  
        on<Every<60,Per<std::chrono::seconds>>, Optional<With<MotionCapture>>,Single>().then([this]
        	(const std::shared_ptr<const MotionCapture>& mocap){
	        std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();    
	        double frame_time_since_start = std::chrono::duration_cast<std::chrono::milliseconds>(now-start_time).count() / float(std::milli::den);  

	        autocal::TimeStamp current_timestamp = start_timestamp + std::chrono::duration_cast<std::chrono::microseconds>(now-start_time).count();
    		
    		if(video_frames * frame_duration < frame_time_since_start){
	            video_frames++;
	        } else {
	            return;
	        }

	        //Add measurement for mocap
	        if(mocap){
		        for(auto& rigidBody : mocap->rigidBodies){
		        	int id = rigidBody.id;
		        	Transform3D pose;
		        	pose.translation() = arma::vec3({double(rigidBody.position[0]),
													 double(rigidBody.position[1]),
													 double(rigidBody.position[2])});
	                UnitQuaternion q(rigidBody.rotation[3], // real part
	                				 arma::vec3({
	                				 	double(rigidBody.rotation[0]),//imaginary part
										double(rigidBody.rotation[1]),
										double(rigidBody.rotation[2])
											 	})
	                				 );
					pose.rotation() = Rotation3D(q);
		        	// pose = Transform3D::createScale(arma::vec3({-1,1,1})) * pose;
		        	if(id != 1){ 
						sensorPlant.mocapRecording.addMeasurement("mocap", current_timestamp, id, pose,false,false);
		        	} else {
						// std::cout << "pose: " << id << " = \n" << pose << std::endl; 
						// std::cout << "quat: " << id << " = " << q.t() << std::endl; 
						// pose.x() = -pose.x();
		        		sensorPlant.setGroundTruthTransform("mocap", "psmove", pose.i());
		        	}
		        }	
	        } else {
	        	std::cout << "NO MOCAP RECEIVED" << std::endl;
	        }


	        //Update and add measurement for psmove
	        psmoveTracker.update();
			if(!use_simulation) psmoveTracker.addMeasurementsToStream(sensorPlant, current_timestamp);

	        window.setActive(true);

	        // // Clear color buffer  
	        glClear(GL_COLOR_BUFFER_BIT);
	        
	        psmoveTracker.render();

	        glMatrixMode(GL_PROJECTION);
	        //49.3 is the measured vFOV of the pseye camera on blue setting
		    glm::mat4 proj =

		        // glm::perspectiveFov<float>(PSEYE_FOV_BLUE_DOT,
		        // image->width, image->height, 0.01f, 10.0f);

		        glm::perspective(   float(49.3 * 3.14159 / 180.0),            //VERTICAL FOV
		                            float(width) / float(height),  //aspect ratio
		                            0.01f,         //near plane distance (min z)
		                            10.0f           //Far plane distance (max z)
		                            );
		    glLoadMatrixf(glm::value_ptr(proj));

	        //TODO: perform matching
	        std::vector<std::pair<int,int>> matches = sensorPlant.matchStreams("psmove","mocap",current_timestamp, 0);

	        drawSensorStreams(sensorPlant, "psmove", current_timestamp, matches);

	        //Get interaction
	        handleInput(window, frame_time_since_start);
	        
	        //Display what we have drawn
	        window.display();

	        //Shutdown if necessary
	        if(!running){
	            powerplant.shutdown();
	        }
        });

		on<Shutdown>().then([this]{
			//Compute final average framerate
			std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();    
		    double finish_time = std::chrono::duration_cast<std::chrono::milliseconds>(now-start_time).count() / float(std::milli::den);     
		    std::cout << "average video framerate = " << double(video_frames) / finish_time << " Hz " << std::endl; 

		});

    }
}
}
