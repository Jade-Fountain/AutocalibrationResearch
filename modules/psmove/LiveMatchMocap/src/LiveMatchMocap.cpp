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
#include "messages/input/proto/MotionCapture.pb.h"
#include "utility/math/geometry/UnitQuaternion.h"
#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/matrix/Transform3D.h"

namespace modules {
namespace psmove {

    using messages::support::Configuration;
    using utility::math::geometry::UnitQuaternion;
	using messages::input::proto::MotionCapture;
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
    window(sf::VideoMode(640*2, 480*2), "OpenGL", sf::Style::Default, sf::ContextSettings(32)),
    sensorPlant(false) //Do not simulate
     {

        on<Configuration>("LiveMatchMocap.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file LiveMatchMocap.yaml
        	width = config["width"].as<int>();
		    height = config["height"].as<int>();
	    	fps = config["fps"].as<float>();
	    	frame_duration = 1.0 / fps;
        });

        on<Startup>().then([this]{

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
  			start_timestamp = std::chrono::duration_cast<std::chrono::microseconds>(start).count()
        });

		   //Main Loop  
        on<Every<60,Per<std::chrono::seconds>>, With<MotionCapture>,Single>().then([this]
        	(const MotionCapture& mocap){
	        std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();    
	        double frame_time_since_start = std::chrono::duration_cast<std::chrono::milliseconds>(now-start_time).count() / float(std::milli::den);  

	        autocal::TimeStamp current_timestamp = start_timestamp + std::chrono::duration_cast<std::chrono::microseconds>(now-start).count();
    		
    		if(video_frames * frame_duration < frame_time_since_start){
	            video_frames++;
	        } else {
	            return;
	        }

	        //Add measurement for mocap
	        for(auto& rigidBody : mocap.rigidBodies){
	        	int id = rigidBody.id;
	        	Transform3D pose;
	        	pose.translation() = rigidBody.position();
                UnitQuaternion q(arma::vec4{rigidBody.rotation().t(),
                							rigidBody.rotation().x(),
                                            rigidBody.rotation().y(),
                                            rigidBody.rotation().z()
                                            });
				pose.rotation() = Rotation3D(q);
				//Reflect coordinate system
				pose.z() = -pose.z();
				sensorPlant.mocapRecording.addMeasurement("mocap", current_timestamp, id, pose);
	        }

	        //Add measurement for psmove
	        psmoveTracker.update();

	        window.setActive(true);

	        // // Clear color buffer  
	        glClear(GL_COLOR_BUFFER_BIT);
	        
	        psmoveTracker.render();

	        //Draw red crosshair for aiming camera
	        drawCrossHair();

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
