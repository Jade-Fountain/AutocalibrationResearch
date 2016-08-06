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
#include "SFML/Window.hpp"

namespace module {
namespace psmove {

    class Play : public NUClear::Reactor {
    	std::chrono::time_point<std::chrono::steady_clock> start;
    	int video_frames;
    	autocal::TimeStamp videoStartTime;
    	CvCapture* video;

		std::string video_filename;

		float frame_duration;
        bool use_simulation = false;

		sf::Window* window;
		// GLFWwindow* window;

		autocal::SensorPlant sensorPlant;

		//TODO: make better callback system
		int psMoveLatency = -1400;
		bool paused = false;
		bool running = true;
        bool first_iter = true;
		void handleInput(const sf::Window& w, double time_since_start);
		

    public:
        /// @brief Called by the powerplant to build and setup the Play reactor.
        explicit Play(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULES_PSMOVE_PLAY_H
