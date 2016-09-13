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

#ifndef MODULES_INPUT_OPENNI_MANAGER_H
#define MODULES_INPUT_OPENNI_MANAGER_H


#include <OpenNI.h>
#include "NiTE.h"

#include <nuclear>
#include <string>

namespace module {
namespace input {

    class OpenNIManager : public NUClear::Reactor {


		float Colors[4][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 1, 1}};
		int colorCount = 3;

		static const int MAX_USERS = 10;
		bool g_visibleUsers[MAX_USERS] = {false};
		nite::SkeletonState g_skeletonStates[MAX_USERS] = {nite::SKELETON_NONE};
        
        nite::UserTracker userTracker;

		openni::RGB888Pixel*		m_pTexMap;
		unsigned int		m_nTexMapX;
		unsigned int		m_nTexMapY;

		std::set<int> excludedJoints;

		void updateUserState(const nite::UserData& user, unsigned long long ts);
   		void userMessage(std::string message, const nite::UserData& user,  unsigned long long ts);
    public:
        /// @brief Called by the powerplant to build and setup the OpenNIManager reactor.
        explicit OpenNIManager(std::unique_ptr<NUClear::Environment> environment);
		
    };

}
}

#endif  // MODULES_INPUT_OPENNI_MANAGER_H
