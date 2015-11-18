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
			float errorThreshold = std::fmax(1e-3,config["angle_stddev"].as<float>() + config["disp_stddev"].as<float>());
        	std::cout << "errorThreshold = " << errorThreshold << std::endl;
        	int fails = 0;
	        if(config["calibration_process_id"].as<int>() == 0){
				for(int j = 0; j < config["number_of_trials"].as<int>(); j++){	
					// simWorldTransform[key] = Transform3D::getRandom(1,0.1);
					Transform3D X = Transform3D::getRandomU(M_PI,0);//transpose because column major reading
					Transform3D Y = Transform3D::getRandomU(M_PI,0);
					

					int N = config["number_of_samples"].as<int>();
					std::vector<Transform3D> samplesA;
					std::vector<Transform3D> samplesB;
					for(int i = 0; i < N; i++){
						//sample B randomly
						Transform3D B = Transform3D::getRandomU(M_PI,1);
						//Noise:
						Transform3D localNoise = Transform3D::getRandomN(config["angle_stddev"].as<float>() , config["disp_stddev"].as<float>() );
						// std::cout << "noise = " << arma::vec4(localNoise * arma::vec4({0,0,0,1})).t() << std::endl;
						Transform3D globalNoise = Transform3D::getRandomN(config["angle_stddev"].as<float>()  ,config["disp_stddev"].as<float>() );
						// Transform3D globalNoise = Transform3D::getRandomN(0.310524198 ,0.052928682);
						// Transform3D noise = Transform3D();
						
						Transform3D A =  Y * globalNoise * B * localNoise * X.i();

						Transform3D check = A*X - Y*B;
						std::cout << "Check AX=YB: \n" << check << std::endl;

						samplesA.push_back(A);
						samplesB.push_back(B);
					}

					bool success = true;

					// std::pair<Transform3D, Transform3D> result = autocal::CalibrationTools::solveZhuang1994(samplesA , samplesB, success);
					std::pair<Transform3D, Transform3D> result = autocal::CalibrationTools::solveKronecker_Shah2013(samplesA , samplesB, success);

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

					std::cout << "X = \n" << X << std::endl;
					std::cout << "measuredX = \n" << measuredX << std::endl;
					std::cout << "Y = \n" << Y << std::endl;
					std::cout << "measuredY = \n" << measuredY << std::endl;
					if((errorX > errorThreshold || errorY > errorThreshold)){
						fails++;

						UnitQuaternion qx(Rotation3D(X.rotation()));
						UnitQuaternion measuredQx(Rotation3D(measuredX.rotation()));
						UnitQuaternion qy(Rotation3D(Y.rotation()));
						UnitQuaternion measuredQy(Rotation3D(measuredY.rotation()));
						
						std::cout << "Rot(qX) = \n" << Rotation3D(qx) << std::endl;
						std::cout << "Rot(qY) = \n" << Rotation3D(qy) << std::endl;
						
						std::cout << "qx =" << qx.t() << " angle = "<< qx.getAngle() << " axis " << qx.getAxis().t() << std::endl;
						std::cout << "measuredQx =" << measuredQx.t() << " angle = "<< measuredQx.getAngle() << " axis " << measuredQx.getAxis().t() << std::endl;
						std::cout << "qy =" << qy.t() << " angle = "<< qy.getAngle() << " axis " << qy.getAxis().t() << std::endl;
						std::cout << "measuredQy =" << measuredQy.t() << " angle = "<< measuredQy.getAngle() << " axis " << measuredQy.getAxis().t() << std::endl;


					}
					std::cout << "Transform3D::norm(measuredX .i() *  X) = " << errorX << std::endl;
					std::cout << "Transform3D::norm(measuredY .i() *  Y) = " << errorY << std::endl;
					std::cout <<  "total error = " << totalError << std::endl;
				}
			} 
			else if(config["calibration_process_id"].as<int>() == 1)
			{
				for(int j = 0; j < config["number_of_trials"].as<int>(); j++){	
					UnitQuaternion q = UnitQuaternion::getRandomU(2*M_PI);
					Rotation3D R(q);
					UnitQuaternion q2(R);
					Rotation3D R2(q2);


					if((arma::norm((q-q2) - arma::vec4({1,0,0,0})) > errorThreshold
					|| Rotation3D::norm(R.t() * R2) > errorThreshold)){
						if(q2.getAngle() > M_PI){
							std::cout << "\n\n\n\nangle > PI: " << q2.getAngle() << std::endl;
						} 
						std::cout << "q = " << (q).t() << " angle = " << q.getAngle() << " axis " << q.getAxis().t() << std::endl;
						std::cout << "q2 = " << (q2).t() << " angle = " << q2.getAngle() << " axis " << q2.getAxis().t() << std::endl;
						std::cout << "q*q2.i() = " << (q-q2).t() << std::endl;
						std::cout << "R-R2 = " << Rotation3D::norm(R.t() * R2) << std::endl;

						fails++;
					}
				}
			} 
			else if(config["calibration_process_id"].as<int>() == 1)
			{
				for(int j = 0; j < config["number_of_trials"].as<int>(); j++){	

				}
			}

			std::cout << "Total failures: " << fails << " / " << config["number_of_trials"].as<int>() << std::endl;


        });

    }

}
}
