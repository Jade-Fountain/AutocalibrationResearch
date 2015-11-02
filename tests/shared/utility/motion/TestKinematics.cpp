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

#include <catch.hpp>

#include "messages/input/ServoID.h"
#include "messages/input/Sensors.h"
#include "utility/motion/ForwardKinematics.h"
#include "utility/motion/InverseKinematics.h"

using messages::input::ServoID;
using messages::input::Sensors;
using utility::motion::kinematics::DarwinModel;
using utility::motion::kinematics::calculateHeadJoints;

TEST_CASE("Test the Head kinematics", "[utility][motion][kinematics][head]") {

    const double ITERATIONS = 10000;

    srand(time(nullptr));

    for(int i = 0; i < ITERATIONS; ++i) {

        // Make a random camera vector
        arma::vec3 camVec = { double(rand()), double(rand()), double(rand()) };
        camVec = arma::normalise(camVec);

        INFO("Testing with the random vector, " << camVec.t());

        std::vector<std::pair<messages::input::ServoID, float>> angles = utility::motion::kinematics::calculateHeadJoints<DarwinModel>(camVec);

        // Make our sensors object
        Sensors sensors;
        sensors.servos = std::vector<Sensors::Servo>(20);

        // Insert our known sensors (from the calculated angles) into our sensors
        for (auto& angle : angles) {
                ServoID servoID;
                float position;

                std::tie(servoID, position) = angle;

                sensors.servos[static_cast<int>(servoID)].presentPosition = position;
        }

        // Do our forward kinematics
        arma::mat44 fKin = utility::motion::kinematics::calculatePosition<DarwinModel>(sensors, ServoID::HEAD_PITCH)[ServoID::HEAD_PITCH];

        // Check that our vector that forward kinematics finds is close to what is expected
        REQUIRE(double(fKin(0, 0) - camVec[0]) == Approx(0));
        REQUIRE(double(fKin(1, 0) - camVec[1]) == Approx(0));
        REQUIRE(double(fKin(2, 0) - camVec[2]) == Approx(0));
    }
}