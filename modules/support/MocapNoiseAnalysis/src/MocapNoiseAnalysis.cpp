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

#include "MocapNoiseAnalysis.h"

#include "messages/support/Configuration.h"

#include "messages/input/MotionCapture.h"

namespace modules {
namespace support {

    using messages::support::Configuration;
    using messages::input::RigidBodyFrame;


    using utility::math::matrix::Transform3D;
    using utility::math::matrix::Rotation3D;
    using utility::math::geometry::UnitQuaternion;

    MocapNoiseAnalysis::MocapNoiseAnalysis(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("MocapNoiseAnalysis.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file MocapNoiseAnalysis.yaml
        });

        on<Trigger<RigidBodyFrame>>().then([this](const RigidBodyFrame& mocap){
        	for(auto& rb : mocap.poses){
    			int rbID = rb.first;
    			Transform3D pose = rb.second;
    			
    			//Rotation measurement
    			//TODO: try axis angle
    			Rotation3D rot = pose.rotation();
    			float rotNorm = Rotation3D::norm(rot);
    			arma::vec rotMeas = {rotNorm};

    			//Position measurement
    			arma::vec3 pos = pose.translation();

    			if(stats.count(rbID) == 0){
    				stats[rbID] = arma::running_stat_vec<arma::vec>(true);
    			}

    			arma::vec measurement = arma::join_cols(rotMeas,pos);
    			stats[rbID](measurement);
        	}
        });

        on<Every<10,std::chrono::seconds>>().then([this]{
        	if(stats.size() == 0){
        		log("WARNING: NO MOCAP RECEIVED");
        	}
        	for(auto rb : stats){
        		log("===================");
        		log("RB ", rb.first);
        		log("mean = \n", rb.second.mean());
        		log("variance = \n", rb.second.cov());
        		log("stddev = \n", rb.second.stddev());
        		log("===================");
        	}
        });
    }
}
}
