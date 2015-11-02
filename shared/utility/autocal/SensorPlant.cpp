/*
author Jake Fountain
This code is part of mocap-kinect experiments*/

#include "SensorPlant.h"
#include "utility/math/matrix/Transform3D.h"
#include <math.h>
#include <set>

namespace autocal {
	using utility::math::matrix::Transform3D;

	//match each rigid body in stream 1 with a rigid body in stream 2
	std::vector<std::pair<int,int>> SensorPlant::matchStreams(std::string stream_name_1, std::string stream_name_2, TimeStamp now, TimeStamp latencyOfStream1){
		// std::cout << "FRAME BEGIN"  << std::endl;
		auto start = std::chrono::high_resolution_clock::now();

		std::vector<std::pair<int,int>> empty_result;

		MocapStream& stream2 = mocapRecording.getStream(stream_name_2);
		
		std::pair<std::string,std::string> hypothesisKey({stream_name_1,stream_name_2});

		//Initialise eliminated hypotheses if necessary
		if(correlators.count(hypothesisKey) == 0){
			correlators[hypothesisKey] = Correlator();
		}
		auto& correlator = correlators[hypothesisKey];

		//Check we have data to compare
		if(stream2.size() == 0){
			return empty_result;
		}

		std::map<MocapStream::RigidBodyID, Transform3D> currentState2 = stream2.getCompleteStates(now);
		std::map<MocapStream::RigidBodyID, Transform3D> currentState1;
		
		//if we simulate the data, derive it from the second stream
		if(simulate){
			currentState1 = stream2.getCompleteSimulatedStates(now + latencyOfStream1, {18,12}, simParams.front());
		} else {
			MocapStream& stream1 = mocapRecording.getStream(stream_name_1);
			if(stream1.size() == 0) return empty_result;
     		currentState1 = stream1.getCompleteStates(now + latencyOfStream1);
		}

		//Update statistics
		for(auto& state1 : currentState1){
			//For each rigid body to be matched
			MocapStream::RigidBodyID id1 = state1.first;
			for(auto& state2 : currentState2){
				//For each rigid body to match to
				MocapStream::RigidBodyID id2 = state2.first;

				correlator.addData(id1, state1.second, id2, state2.second);	
			}
		}

		//Compute correlations
		if(correlator.sufficientData()){
			correlator.compute();
		}

		// std::cout << "FRAME END"  << std::endl;

		std::vector<std::pair<int,int>> correlations = correlator.getBestCorrelations();

		auto finish = std::chrono::high_resolution_clock::now();
		computeTimes(double(std::chrono::duration_cast<std::chrono::nanoseconds>(finish-start).count() * 1e-6));
		
		//Compute correct guesses:
		for (auto& cor : correlations){
			if(correctGuesses.count(cor.first) == 0) correctGuesses[cor.first] = 0;
			if(totalGuesses.count(cor.first) == 0) totalGuesses[cor.first] = 0;
			correctGuesses[cor.first] += int(cor.first == 0 && cor.second == 2);
			totalGuesses[cor.first] ++;
		}
		


		return correlations;

	}

	std::map<MocapStream::RigidBodyID,float> SensorPlant::multiply(std::map<MocapStream::RigidBodyID,float> m1, std::map<MocapStream::RigidBodyID,float> m2){
		std::map<MocapStream::RigidBodyID,float> result;
		float learningRate = 0.1;
		for(auto& x : m1){
			if(m2.count(x.first) != 0){
				//Exponential filter
				// result[x.first] = (1-learningRate) * m1[x.first] + learningRate * m2[x.first];

				//Probabilistic decay filter
				result[x.first] = m1[x.first] * std::pow(m2[x.first],1-learningRate);
			} else {
				result[x.first] = m1[x.first];
			}
		}
		//Add in keys present in m2 but not m1
		for(auto& x : m2){
			if(m1.count(x.first) == 0){
				result[x.first] = m2[x.first];
			}
		}
		return result;
	}

	void SensorPlant::setGroundTruthTransform(std::string streamA, std::string streamB, Transform3D mapAtoB, bool useTruth){
		groundTruthTransforms[std::make_pair(streamA, streamB)] = mapAtoB;
		//HACK CORRECTION (for kinect, no longer necessary)
		// groundTruthTransforms[std::make_pair(streamA, streamB)].translation() += arma::vec3{-0.38,0,0};
		// std::cout << "groundTruthTransforms \n" << groundTruthTransforms[std::make_pair(streamA, streamB)]<<  std::endl;

		if(useTruth){
			convertToGroundTruth(streamA, streamB);
			//Set to identity
			groundTruthTransforms[std::make_pair(streamA, streamB)] = Transform3D();
		}
	}

	void SensorPlant::convertToGroundTruth(std::string streamA, std::string streamB){
		auto key = std::make_pair(streamA, streamB);
		
		if(groundTruthTransforms.count(key) != 0 && mocapRecording.streamPresent(streamA)){
			//Get the transform between coordinate systems
			Transform3D streamToDesiredBasis = groundTruthTransforms[key];

			for(auto& frame : mocapRecording.getStream(streamA).frameList()){
				//Loop through and record transformed rigid body poses
				for (auto& rb : frame.second.rigidBodies){
					Transform3D T = streamToDesiredBasis * rb.second.pose;
					rb.second.pose = T;
				}
			}
		} else {
			std::cout << "WARNING: ATTEMPTING TO ACCESSING GROUND TRUTH WHEN NONE EXISTS!!!" << std::endl;
		}


	}

	
	autocal::MocapStream::Frame SensorPlant::getGroundTruth(std::string stream, std::string desiredBasis, TimeStamp now){
		//If we are transforming to the same reference basis, just return the current frame unaltered
		if(stream.compare(desiredBasis) == 0){
			return mocapRecording.getStream(stream).getFrame(now);
		}
		//Init result otherwise
		autocal::MocapStream::Frame truth;
		//make the key to retrieve the ground truth transform
		//TODO: check alternate key ordering too (as its just a matrix inverse to swap order)
		auto key = std::make_pair(stream, desiredBasis);
		if(groundTruthTransforms.count(key) != 0 && mocapRecording.streamPresent(stream)){
			//Get the transform between coordinate systems
			Transform3D streamToDesiredBasis = groundTruthTransforms[key];

			//Get the latest data 
			MocapStream::Frame latestFrame = mocapRecording.getStream(stream).getFrame(now);

			//Loop through and record transformed rigid body poses
			for (auto& rb : latestFrame.rigidBodies){
				truth.rigidBodies[rb.first].pose = streamToDesiredBasis * rb.second.pose;
			}
		} else {
			std::cout << "WARNING: ATTEMPTING TO ACCESS GROUND TRUTH WHEN NONE EXISTS!!!" << std::endl;
		}

		return truth;
	}

	void SensorPlant::next(){
		for(auto& c : correlators){
			c.second.reset();
		}
		if(simParams.size()!=0){
			MocapStream::SimulationParameters s = simParams.front();
			simParams.pop();
			std::cerr << "Finished simulating: " << s.latency_ms << " " << s.noise.angle_stddev << " " << s.noise.disp_stddev << " " 
					  << s.slip.disp.f << " " << s.slip.disp.A << " "
					  << s.slip.angle.f << " " << s.slip.angle.A << " ";
		}
		std::cerr << " Fraction correct: " << std::endl; 
		for(auto guess : correctGuesses){
			std::cerr << "id: " << guess.first << " = " <<  float(guess.second) / float(totalGuesses[guess.first]) << std::endl;
		}
		std::cerr <<  " time= "<< computeTimes.min() << " " << computeTimes.mean() << " " << computeTimes.max() << std::endl;
		correctGuesses.clear();
		totalGuesses.clear();
		computeTimes.reset();
	}

	void SensorPlant::setSimParameters(
		MocapStream::SimulationParameters a1, MocapStream::SimulationParameters a2, int aN,
		MocapStream::SimulationParameters d1, MocapStream::SimulationParameters d2, int dN){
		
		simParams = std::queue<MocapStream::SimulationParameters>();//clear queue
		
		MocapStream::SimulationParameters aStep;	
		if(aN != 1){
			aStep = (a2 - a1) * (1 / float(aN-1));	
		}

		MocapStream::SimulationParameters dStep;
		if(dN != 1){
			dStep = (d2 - d1) * (1 / float(dN-1));
		}

		for(int i = 0; i < aN; i++){
			MocapStream::SimulationParameters a;
			a = a1 + aStep * i;
			for(int j = 0; j < dN; j++){
				MocapStream::SimulationParameters d;
				d = d1 + dStep * j;

				simParams.push(a+d);
			}
		}
	}

















}
