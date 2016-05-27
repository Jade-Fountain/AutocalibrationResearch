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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_VISION_OBSTACLEDETECTOR_H
#define MODULES_VISION_OBSTACLEDETECTOR_H


#include <nuclear>
namespace module {
namespace vision {

    class ObstacleDetector : public NUClear::Reactor {
    public:
        /// @brief Called by the powerplant to build and setup the ObstacleDetector reactor.
        explicit ObstacleDetector(std::unique_ptr<NUClear::Environment> environment);
    };

}
}


#endif