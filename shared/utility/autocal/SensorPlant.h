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
#include "Simulation.h"

#ifndef AUTOCAL_SENSOR_PLANT
#define AUTOCAL_SENSOR_PLANT

namespace autocal {
	
	class SensorPlant{

		using Hypothesis = std::pair<int,int>;
		using NamePair = std::pair<std::string,std::string>;
				
		std::map<NamePair, utility::math::matrix::Transform3D> groundTruthTransforms;

		std::map<NamePair ,Correlator> correlators;

		//State variables
		bool simulate;
		std::queue<SimulationParameters> simParams;
		arma::running_stat<double> computeTimes;
		std::map<int, int> correctGuesses; 
		std::map<int, int> totalGuesses; 
		std::map<int, int> simulatedCorrelations;

		//Simulation parameters
		std::map<Hypothesis, utility::math::matrix::Transform3D> simWorldTransform;
		std::map<Hypothesis, utility::math::matrix::Transform3D> simLocalTransform;


	public:
		SensorPlant(bool sim = false): simulate(sim){}

		bool isSimulated(){return simulate;}

		MocapRecording mocapRecording;

		const MocapStream& getStream(std::string name){
			return mocapRecording.getStream(name);
		}

		void addStream(const MocapStream& s){
			mocapRecording.getStream(s.name()) = s;
		}

		bool streamNotEmpty(std::string name){
			return !getStream(name).isEmpty();
		}

		bool isRunning(){return !simulate || simParams.size()!=0;}
				
		std::vector<Hypothesis> getCorrelations(std::string stream_name_1, std::string stream_name_2, TimeStamp now);
		
		std::vector<Hypothesis> getCorrelationsOfInvariants(std::string stream_name_1, std::string stream_name_2, TimeStamp now);
		
		std::vector<Hypothesis> matchStreams(std::string stream_name_1, std::string stream_name_2, TimeStamp now, TimeStamp latencyOfStream1 = 0);

		std::map<MocapStream::RigidBodyID,float> multiply(std::map<MocapStream::RigidBodyID,float> m1, std::map<MocapStream::RigidBodyID,float> m2);

		void setGroundTruthTransform(std::string streamA, std::string streamB, utility::math::matrix::Transform3D mapAtoB, bool useTruth = false);
		
		void setAnswers(std::map<int,int> answers);
		
		void setSimParameters(
			SimulationParameters a1, SimulationParameters a2, int aN,
			SimulationParameters d1, SimulationParameters d2, int dN);

		bool next();

		void convertToGroundTruth(std::string streamA, std::string streamB);

		autocal::MocapStream::Frame getGroundTruth(std::string stream, std::string desiredBasis, TimeStamp now);

		//TODO:implement
		std::map<MocapStream::RigidBodyID, utility::math::matrix::Transform3D> getCompleteSimulatedStates(TimeStamp now, std::map<int,int> ids, const SimulationParameters& sim, MocapStream& stream);


	};

}
#endif