/*
author Jake Fountain
This code is part of mocap-kinect experiments.
The sensor plant is responsible for fusing multiple measurements*/
#include <armadillo>
#include <chrono>
#include <string>
#include <set>
#include <queue>
#include <map>
#include "MocapStream.h"
#include "MocapRecording.h"
#include "CalibrationTools.h"
#include "Correlator.h"

#ifndef AUTOCAL_SENSOR_PLANT
#define AUTOCAL_SENSOR_PLANT

namespace autocal {
	
	class SensorPlant{
				
		std::map<std::pair<std::string,std::string>, utility::math::matrix::Transform3D> groundTruthTransforms;

		std::map<std::pair<std::string,std::string> ,Correlator> correlators;

		bool simulate;
		std::queue<MocapStream::SimulationParameters> simParams;
		arma::running_stat<double> computeTimes;
		std::map<int, int> correctGuesses; 
		std::map<int, int> totalGuesses; 


	public:
		SensorPlant(bool sim = false): simulate(sim){}

		bool isSimulated(){return simulate;}

		MocapRecording mocapRecording;

		void addStream(const std::string& name, const MocapStream& s){
			mocapRecording.getStream(name) = s;
		}

		bool isRunning(){return !simulate || simParams.size()!=0;}
				
		std::vector<std::pair<int,int>> getCorrelations(std::string stream_name_1, std::string stream_name_2, TimeStamp now);
		
		std::vector<std::pair<int,int>> getCorrelationsOfInvariants(std::string stream_name_1, std::string stream_name_2, TimeStamp now);
		
		std::vector<std::pair<int,int>> matchStreams(std::string stream_name_1, std::string stream_name_2, TimeStamp now);

		std::map<MocapStream::RigidBodyID,float> multiply(std::map<MocapStream::RigidBodyID,float> m1, std::map<MocapStream::RigidBodyID,float> m2);

		void setGroundTruthTransform(std::string streamA, std::string streamB, utility::math::matrix::Transform3D mapAtoB, bool useTruth = false);
		
		void setSimParameters(
			MocapStream::SimulationParameters a1, MocapStream::SimulationParameters a2, int aN,
			MocapStream::SimulationParameters d1, MocapStream::SimulationParameters d2, int dN);

		void next();

		void convertToGroundTruth(std::string streamA, std::string streamB);

		std::map<int, utility::math::matrix::Transform3D> getGroundTruth(std::string stream, std::string desiredBasis, TimeStamp now);

	};

}
#endif