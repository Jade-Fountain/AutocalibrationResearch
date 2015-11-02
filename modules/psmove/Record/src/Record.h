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


namespace modules {
namespace psmove {

    class Record : public NUClear::Reactor {
		Tracker psmoveTracker;
		sf::Window window;
		int width;
		int height;
		int video_frames;
		bool running = true;
    	std::chrono::time_point<std::chrono::system_clock> start_time;
		CvVideoWriter *writer;
		
		void handleInput(const sf::Window& w, double time_since_start);

    public:
        /// @brief Called by the powerplant to build and setup the Record reactor.
        explicit Record(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULES_PSMOVE_RECORD_H
