/*
 * This file is part of NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2015 NUbots <nubots@nubots.net>
 */

#ifndef MODULES_PSMOVE_PLAY_H
#define MODULES_PSMOVE_PLAY_H
#include <nuclear>

#include "utility/autocal/MocapStream.h"
#include "utility/autocal/SensorPlant.h"


#ifdef __APPLE__
    #include <GL/glew.h>  

    #include <GLFW/glfw3.h>  
    #include <GLUT/glut.h>
#else
  #include <GL/gl.h>
  #include <GL/glut.h>
#endif 

#include "opencv2/opencv.hpp"

namespace modules {
namespace psmove {

    class Play : public NUClear::Reactor {
    	std::chrono::time_point<std::chrono::steady_clock> start;
    	int video_frames;
    	autocal::TimeStamp videoStartTime;
    	CvCapture* video;

		std::string video_filename;

		float frame_duration;

		GLFWwindow* window;

		autocal::SensorPlant sensorPlant;

		//TODO: make better callback system
		static autocal::TimeStamp psMoveLatency;
		static bool paused;
 		static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods); 
		

    public:
        /// @brief Called by the powerplant to build and setup the Play reactor.
        explicit Play(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULES_PSMOVE_PLAY_H
