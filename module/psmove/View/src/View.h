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

#ifndef MODULES_PSMOVE_RECORD_H
#define MODULES_PSMOVE_RECORD_H

#ifdef __APPLE__
    // #include <OpenGL/gl.h>
    #include <GL/glew.h>  

    #include <GLFW/glfw3.h>  
    #include <GLUT/glut.h>
#else
  #include <GL/gl.h>
  #include <GL/glut.h>
#endif 
#include <nuclear>
#include <chrono>
#include "utility/autocal/PSMoveUtils.h"

#include "SFML/Window.hpp"


namespace module {
namespace psmove {

    class View : public NUClear::Reactor {
		//Psmove object for getting psmove pose and video
		Tracker psmoveTracker;
		
		//Window object to draw to
		std::unique_ptr<sf::Window> window;
		
		//Video output properties
		float fps;
		float frame_duration;
		int width;
		int height;

		//State variables
		int video_frames = 0;
		bool running = true;
    	std::chrono::time_point<std::chrono::system_clock> start_time;
		
		//handle keyboard
		void handleInput(const sf::Window& w, double time_since_start);

    public:
        /// @brief Called by the powerplant to build and setup the View reactor.
        explicit View(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULES_PSMOVE_RECORD_H
