/*
author Jake Fountain
This code is part of mocap-kinect experiments*/
#include <armadillo>
#include <chrono>
#include <dirent.h>
#include <map>
#include <set>
#include "MocapStream.h"
#include "CalibrationTools.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/geometry/UnitQuaternion.h"

#ifndef AUTOCAL_CORRELATOR
#define AUTOCAL_CORRELATOR

namespace autocal {

	class Correlator
	{
	public:
		Correlator();
		~Correlator(){};
		
	private:
		//CONFIG
		int number_of_samples;

		float difference_threshold;

		float elimination_score_threshold; 

		float score_inclusion_threshold;
		
		//STATE
		using Stream = std::vector<utility::math::matrix::Transform3D>;

		class RigidBodyInfo{
		public:
			RigidBodyInfo(int id_) : id(id_){

			}

			//Details of the stream which links to one of the hypotheses
			int id;
			Stream states;

			//Hypothesis streams
			std::map<int, Stream> hypotheses;

			//Latest scores
			std::map<int, float> scores;

			//Ready for scoring?
			bool computable = false;

			//Flag for reset
			std::vector<bool> initialised;
		};

		//Central state of correlator
		std::map<int, RigidBodyInfo> rigidBodies;

		float getSylvesterScore(const Stream& states1, const Stream& states2);

		float getRotationScore(const Stream& states1, const Stream& states2);
		
		void resetRecordedStates();

		bool stateIsNew(const utility::math::matrix::Transform3D& T, const Stream& states);
		
	public:

		void addData(int id1, utility::math::matrix::Transform3D T1, int id2, utility::math::matrix::Transform3D T2);

		void eliminateAndNormalise(std::map<int,float> totalScores);

		std::map<int, bool> sufficientData();

		void compute(const std::map<int, bool>& streamsReady);

		void reset();

		std::vector<std::pair<int,int>> getBestCorrelations();

		float likelihood(float error){
			return std::exp(-error * error);
		}

	};


}
#endif