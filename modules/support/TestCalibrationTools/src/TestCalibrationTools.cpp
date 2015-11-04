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

#include "TestCalibrationTools.h"

#include <iostream>

#include <vector>

#include <chrono>
#include <iostream>


#include "opencv2/core/core_c.h"
#include "opencv2/highgui/highgui_c.h"


#include "utility/autocal/CalibrationTools.h"
#include "messages/support/Configuration.h"
#include "utility/math/geometry/UnitQuaternion.h"
#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/matrix/Transform3D.h"

namespace modules {
namespace support {

    using messages::support::Configuration;
    using utility::math::geometry::UnitQuaternion;
    using utility::math::matrix::Rotation3D;
    using utility::math::matrix::Transform3D;

    TestCalibrationTools::TestCalibrationTools(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("TestCalibrationTools.yaml").then([this] (const Configuration& config) {
			float errorThreshold = config["angle_stddev"].as<float>() + config["disp_stddev"].as<float>();
        	std::cout << "errorThreshold = " << errorThreshold << std::endl;
        	int fails = 0;
			for(int j = 0; j < config["number_of_trials"].as<int>(); j++){	
				// simWorldTransform[key] = Transform3D::getRandom(1,0.1);
				Transform3D X = Transform3D::getRandomU(1,1);//transpose because column major reading
				Transform3D Y = Transform3D::getRandomU(1,0.1);
				

				int N = config["number_of_samples"].as<int>();
				std::vector<Transform3D> samplesA;
				std::vector<Transform3D> samplesB;
				for(int i = 0; i < N; i++){
					//sample B randomly
					Transform3D B = Transform3D::getRandomU(1,1);
					//Noise:
					Transform3D localNoise = Transform3D::getRandomN(config["angle_stddev"].as<float>() , config["disp_stddev"].as<float>() );
					// std::cout << "noise = " << arma::vec4(localNoise * arma::vec4({0,0,0,1})).t() << std::endl;
					Transform3D globalNoise = Transform3D::getRandomN(config["angle_stddev"].as<float>()  ,config["disp_stddev"].as<float>() );
					// Transform3D globalNoise = Transform3D::getRandomN(0.310524198 ,0.052928682);
					// Transform3D noise = Transform3D();
					
			 	 	
					// std::cout << "slippage[" << rbID << "] = " << Transform3D::norm(slippage[rbID]) << std::endl;
					// std::cout << "noise[" << rbID << "] = " << Transform3D::norm(noise) << std::endl;
					// std::cout << "slippage/noise[" << rbID << "] = " << Transform3D::norm(slippage[rbID])/Transform3D::norm(noise) << std::endl;

					Transform3D A =  Y * globalNoise * B * localNoise * X.i();

					samplesA.push_back(A);
					samplesB.push_back(B);
				}

				bool success = true;
				std::pair<Transform3D, Transform3D> result = autocal::CalibrationTools::solveHomogeneousDualSylvester(samplesA , samplesB, success);

				auto measuredX = result.first;
				auto measuredY = result.second;

				
				float errorX = Transform3D::norm(measuredX.i() * X);
				float errorY = Transform3D::norm(measuredY.i() * Y);

				float totalError = 0;
				for(int i = 0; i < samplesA.size(); i++){
					const Transform3D& A = samplesA[i];
					const Transform3D& B = samplesB[i];
					Transform3D errorMat = (A * measuredX).i() * (measuredY * B);
					float error = Transform3D::norm(errorMat);
					totalError += error;
					// std::cout << "Error = " << error << " for transform\n" << errorMat << std::endl;
				}

				if(errorX > errorThreshold || errorY > errorThreshold){
					fails++;
					std::cout << "X = \n" << X << std::endl;
					std::cout << "measuredX = \n" << measuredX << std::endl;
					std::cout << "Y = \n" << Y << std::endl;
					std::cout << "measuredY = \n" << measuredY << std::endl;


					std::cout << "Transform3D::normmeasuredX .i() *  X) = " << errorX << std::endl;
					std::cout << "Transform3D::normmeasuredY .i() *  Y) = " << errorY << std::endl;
					std::cout <<  "total error = " << totalError << std::endl;
				}
			}
			std::cout << "Total failures: " << fails << " / " << config["number_of_trials"].as<int>() << std::endl; 

        });

    }
}
}
