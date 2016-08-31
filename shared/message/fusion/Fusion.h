/*
 * This file is part of the Autocalibration Codebase.
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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MESSAGES_INPUT_FUSION_H
#define MESSAGES_INPUT_FUSION_H

#include <string>
#include "utility/math/matrix/Transform3D.h"

namespace message {
namespace fusion {


	const std::string OPENNI_STREAM = "openni";
	const std::string MOCAP_STREAM = "mocap";
	const std::string PSMOVE_STREAM = "psmove";

    class MatchResults{
    public:
        std::string stream1;
        std::string stream2;

        std::vector<std::pair<int, int>> matches;


        //Returns 
        int getMatchFor(std::string stream, int id) const{
        	if(stream == stream1){
        		for(auto& match : matches){
        			if(id == match.first) return match.second;
        		}
        	} else if(stream == stream2){
        		for(auto& match : matches){
        			if(id == match.second) return match.first;
        		}
        	} else {
        		std::cout << "WARNING: Matches not found for stream " << stream << std::endl;
        	}
        	//No match found
        	return -1;
        }	
    };

}
}

#endif
