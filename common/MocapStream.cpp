/*
author Jake Fountain
This code is part of mocap-kinect experiments*/

#include "MocapStream.h"

namespace autocal {

	using utility::math::matrix::Transform3D;
	using utility::math::matrix::Rotation3D;
	using utility::math::geometry::UnitQuaternion;


	std::string MocapStream::Frame::toString(){
		std::stringstream os;
		for(auto& x : rigidBodies){
			os << "rigidBodies[" << int(x.first) << "] = \n" << x.second.pose << std::endl;
		}
		os << "=======================";
		return os.str();
	}

	std::string MocapStream::toString(){
		std::stringstream os;
		os << "Mocap Stream: " << name()  << " || Size: " << size() << std::endl;
		for(auto& x : stream){
			os << "stream[" << int(x.first) << "] = \n" << x.second.toString() << std::endl;
		}
		return os.str();
	}


	MocapStream::Frame MocapStream::createFrame(arma::mat m){
		Frame f;
		for(int n = 0; n < m.n_cols; n++){
			arma::vec data = m.col(n);
			
			RigidBody r;
			
			arma::vec3 pos = data.rows(1,3);
			//Change back to mocap coords from nubots coords (sigh...)
			r.pose.translation() = arma::vec3{-pos[1],pos[2],-pos[0]};
			
			Rotation3D rot;
			int start = 4;
			for(int i = 0; i < 3; i++){
				rot.row(i) = data.rows(start + 3 * i, start + 3 * i + 2).t();
			}
			UnitQuaternion q(rot);
			//Change back to mocap coords from nubots coords (sigh...)
			UnitQuaternion q_(arma::vec4{
				 q.kX(),
				-q.kZ(),
				 q.kW(),
				-q.kY(),
				});
			//Turn back into rotation
			r.pose.rotation() = Rotation3D(q_);

			// std::cout << "data: " <<  data << std::endl;
			// std::cout << "id:" << int(data[0]) << std::endl;
			// std::cout << "position:" << r.position << std::endl;
			// std::cout << "rotation:" << r.rotation << std::endl;
			
			f.rigidBodies[int(data[0])] = r;
		}
		return f;
	}

	MocapStream::Frame MocapStream::getFrame(const std::chrono::system_clock::time_point& t){
		//Get last frame at current time point
		return getFrame(getTimeStamp(t));
	}
	
	MocapStream::Frame MocapStream::getInterpolatedFrame(const TimeStamp& t){
		//Get last frame at current time point
		if(stream.count(t) != 0){
			return stream[t];
		} else {
			const auto ub = stream.lower_bound(t);
			// auto ub = stream.upper_bound(t);
			if(ub == stream.begin()) return ub->second;
			auto lb = ub;
			lb--;
			if(ub == stream.end()) return lb->second;
			float alpha = float(t - lb->first)/float(ub->first - lb->first);
			return Frame::interpolate(lb->second, ub->second, alpha);
		}
	}
	
	MocapStream::Frame MocapStream::getFrame(const TimeStamp& t){
		//Get last frame at current time point
		if(stream.count(t) != 0){
			return stream[t];
		} else {
			auto ub = stream.lower_bound(t);
			if(ub == stream.begin()) return ub->second;
			ub--;
			return ub->second;
		}
	}
	TimeStamp MocapStream::getFrameTime(const TimeStamp& t){
		if(stream.count(t) != 0){
			return t;
		} else {
			return stream.lower_bound(t)->first;
		}
	}

	std::map<TimeStamp,MocapStream::Frame>::iterator MocapStream::getUpperBoundIter(const TimeStamp& t){
		//Get last frame at current time point
		return stream.upper_bound(t);
	}
	
	std::map<TimeStamp,MocapStream::Frame>::iterator MocapStream::getLowerBoundIter(const TimeStamp& t){
		//Get last frame at current time point
		return stream.lower_bound(t);
	}
	
	bool MocapStream::loadMocapData(std::string folder_path, const TimeStamp& start_time, const std::chrono::system_clock::time_point& end_time){
		std::cout << "Loading data ..." << std::endl;
		std::cerr << "Loading data ..." << std::endl;

		TimeStamp min = start_time;
		TimeStamp max = getTimeStamp(end_time);

		bool success = true;

		DIR* dir = opendir(folder_path.c_str());
		std::cout << "Folder opened ..." << std::endl;
		dirent * file;
		TimeStamp max_loaded = min;
		TimeStamp min_loaded = max;

		while ((file = readdir(dir)) != NULL){
			
			std::string filename = file->d_name;
			// std::cout << "Filename = " << filename << std::endl;
			TimeStamp timestamp;
			try{
				timestamp = std::stoll(filename);
			} catch(std::invalid_argument) {
				std::cout << "Skipping file " << filename << std::endl;
				continue;
			}
			if(timestamp < max && timestamp > min){
				arma::mat frame;

				std::stringstream path;
				path << folder_path << "/" << filename;
				success = success && frame.load(path.str(), arma::arma_binary);

				if(success){ 
					//Do not store frame if it has no info
					if(frame.n_cols!=0){
						stream[timestamp] = createFrame(frame);
					}
				} else {
					continue;
				}

				max_loaded = std::max(max_loaded, timestamp);
				min_loaded = std::min(min_loaded, timestamp);
			}

		}
		if(max != min_loaded && min != max_loaded){
			float period_sec = float(max_loaded - min_loaded) * 1e-6;
			std::cout << "Loaded data from " << min_loaded << " to " << max_loaded << ". i.e. " 
					  << int(period_sec) << " seconds at " << stream.size() / period_sec << "Hz measurement frequency" << std::endl; 
		}

	   	(void)closedir(dir);
		std::cout << "Loading finished " << (success ? "successfully" : "UNSUCCESSFULLY") << std::endl;


		return success;
	}

	bool MocapStream::setRigidBodyInFrame(const std::chrono::system_clock::time_point& frame_time, const unsigned int& id, const Transform3D& pose){
		//Check that the frame doesn't already exist
		TimeStamp t = getTimeStamp(frame_time);
		if(stream.count(t) == 0){
			stream[t] = Frame();
		}
		stream[t].rigidBodies[id] = RigidBody({pose});
	}

	bool MocapStream::setRigidBodyInFrame(const TimeStamp& frame_time, const unsigned int& id, const Transform3D& pose){
		//Check that the frame doesn't already exist
		TimeStamp t = frame_time;
		if(stream.count(t) == 0){
			stream[t] = Frame();
		}
		stream[t].rigidBodies[id] = RigidBody({pose});
	}

	std::map<MocapStream::RigidBodyID, arma::vec> MocapStream::getInvariates(TimeStamp now){
		std::map<MocapStream::RigidBodyID, arma::vec> invariates;
		if(stream.size() != 0){
			
			auto initial = stream.upper_bound(streamStart);
			if(initial == stream.end()){
				return invariates;
			}
			Frame firstFrame = initial->second;
			Frame latestFrame = getFrame(now);

			for (auto& rb : firstFrame.rigidBodies){
				auto rbID = rb.first;
				auto initialTransform = rb.second.pose;
				if(latestFrame.rigidBodies.count(rbID)!=0){

 					auto latestTransform = latestFrame.rigidBodies[rbID].pose;
					//TODO generalise to other sensors and invariates
 					Rotation3D rotation = latestTransform.rotation().i() * initialTransform.rotation();
 					UnitQuaternion q(rotation);
 					float rotationMagnitude = q.getAngle();

					arma::vec3 displacement = latestTransform.translation() - initialTransform.translation();
					invariates[rbID] = arma::vec{rotationMagnitude};

				}
			}
		}

		return invariates;
	}

	std::map<MocapStream::RigidBodyID, arma::vec> MocapStream::getStates(TimeStamp now){
		std::map<MocapStream::RigidBodyID, arma::vec> states;
		
		if(stream.size() != 0){
			
			Frame latestFrame = getFrame(now);

			for (auto& rb : latestFrame.rigidBodies){
				auto rbID = rb.first;
				auto transform = rb.second.pose;

				states[rbID] = transform.translation();
			}
		}

		return states;
	}


	std::map<MocapStream::RigidBodyID, Transform3D> MocapStream::getCompleteStates(TimeStamp now){
		std::map<MocapStream::RigidBodyID, Transform3D> states;
		
		if(stream.size() != 0){
			
			Frame latestFrame = getFrame(now);

			for (auto& rb : latestFrame.rigidBodies){
				auto rbID = rb.first;
				auto transform = rb.second.pose;

				states[rbID] = transform;
			}
		}

		return states;
	}


	std::map<MocapStream::RigidBodyID, arma::vec> MocapStream::getSimulatedStates(TimeStamp now, std::vector<RigidBodyID> ids){
		std::map<MocapStream::RigidBodyID, arma::vec> states;
		
		//TODO:make this better
		Transform3D worldTransform; //M
		worldTransform = worldTransform.rotateY(1);
		worldTransform = worldTransform.translateX(1);
		worldTransform = worldTransform.rotateZ(0.4);
		worldTransform = worldTransform.translateY(0.1);
		
		//TODO: 
		Transform3D localTransform; //L^-1
		localTransform = localTransform.translateX(1);
		localTransform = localTransform.rotateX(-0.4);
		localTransform = localTransform.translateY(0.1);

		if(stream.size() != 0){
			
			Frame latestFrame = getFrame(now);
			RigidBodyID i = 1;
			for (auto& rbID : ids){
				Transform3D transform = worldTransform * latestFrame.rigidBodies[rbID].pose * localTransform;

				states[i++] = transform.translation();
			}
		}

		return states;
	}


	std::map<MocapStream::RigidBodyID, Transform3D> MocapStream::getCompleteSimulatedStates(TimeStamp now, std::vector<RigidBodyID> ids, const SimulationParameters& sim){
		std::map<MocapStream::RigidBodyID, Transform3D> states;

		int lag_milliseconds = sim.latency_ms;
		now -= lag_milliseconds * 1000;
		
		if(stream.size() != 0){
			
			Frame latestFrame = getFrame(now);
			RigidBodyID i = 1;
			for (auto& rbID : ids){
				std::pair<int,int> key = {i,int(rbID)};

				if(simWorldTransform.count(key) == 0){
					// simWorldTransform[key] = Transform3D::getRandom(1,0.1);
					simWorldTransform[key] = Transform3D({ 0.1040,  -0.0023,  -0.9946,  -0.3540,
													      -0.1147,   0.9933,  -0.0143,  -0.9437,
													       0.9879,   0.1156,   0.1030,   1.2106,
													            0,        0,        0,   1.0000}).t();//transpose because column major reading
					std::cout << "simWorldTransform = \n" << simWorldTransform[key] << std::endl;
				}
				if(simLocalTransform.count(key) == 0){
					simLocalTransform[key] = Transform3D::getRandomU(1,0.1);
					std::cout << "simLocalTransform = \n" << simLocalTransform[key] << std::endl;
				}
				//Noise:
				Transform3D localNoise = Transform3D::getRandomN(sim.noise.angle_stddev ,sim.noise.disp_stddev);
				// std::cout << "noise = " << arma::vec4(localNoise * arma::vec4({0,0,0,1})).t() << std::endl;
				Transform3D globalNoise = Transform3D::getRandomN(sim.noise.angle_stddev ,sim.noise.disp_stddev);
				// Transform3D globalNoise = Transform3D::getRandomN(0.310524198 ,0.052928682);
				// Transform3D noise = Transform3D();
				
		 	 	if(slippage.count(rbID) == 0){
		 	 		slippage[rbID] = Transform3D();
		 	 	}
		 	 	float df = sim.slip.disp.f;
		 	 	float displacement = sim.slip.disp.A;

		 	 	float af = sim.slip.angle.f;
		 	 	float angleAmp = sim.slip.angle.A;

		 	 	float x = displacement * std::sin(2 * M_PI * now * 1e-6 * df);
		 	 	float angle = angleAmp * std::sin(2 * M_PI * now * 1e-6 * af);

				slippage[rbID] = Transform3D(Rotation3D::createRotationZ(angle),arma::vec3({x,0,0}));
				// std::cout << "slippage[" << rbID << "] = " << Transform3D::norm(slippage[rbID]) << std::endl;
				// std::cout << "noise[" << rbID << "] = " << Transform3D::norm(noise) << std::endl;
				// std::cout << "slippage/noise[" << rbID << "] = " << Transform3D::norm(slippage[rbID])/Transform3D::norm(noise) << std::endl;

				Transform3D transform = simWorldTransform[key] * latestFrame.rigidBodies[rbID].pose * globalNoise * simLocalTransform[key] * slippage[rbID] * localNoise;
				// std::cout << "transform = " << transform.translation().t() << std::endl;

				states[i++] = transform;
			}
		}

		return states;
	}



}