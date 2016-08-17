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
 * Copyright 2016 Autocalibration <nubots@nubots.net>
 */

#include "OpenNI.h"


namespace module {
namespace input {

    OpenNI::OpenNI(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment))
    {

    	on<Startup>().then("OpenNI Start",[this](){
	    	//Startup

    	});

		on<Every<30, Per<std::chrono::seconds>>>().then("OpenNI Read loop",[this](){
			log("main loop running");
		});
    }
    
    OpenNI::~OpenNI(){
    	
    }

}
}
