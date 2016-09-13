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

		float difference_threshold_angle;
		float difference_threshold_pos;

		float elimination_score_threshold; 

		float score_inclusion_threshold;
		//STATE

		using Hypothesis = std::pair<MocapStream::RigidBodyID, MocapStream::RigidBodyID>;
		using Stream = std::vector<utility::math::matrix::Transform3D>;
		using StreamPair = std::pair<Stream, Stream>;


		//Results of sylvester calibrations
		std::map<Hypothesis, utility::math::matrix::Transform3D> bestTransforms;

		//States for matchStreams
		std::map<Hypothesis, StreamPair> recordedStates;
		//Stores scores for the matchings
		std::map<Hypothesis, float> scores;
		//Stores the matches which have been deduced incorrect
		std::set<Hypothesis> eliminatedHypotheses;
		
		std::set<Hypothesis> computableStreams;
		
		float getSylvesterScore(const Stream& states1, const Stream& states2, 
								Hypothesis key);

		float getRotationScore(const Stream& states1, const Stream& states2, 
								Hypothesis key);
		
		void resetRecordedStates();

		void resetStates(int id);

		bool stateIsNew(const utility::math::matrix::Transform3D& T, const Stream& states);
		
	public:

		void addData(MocapStream::RigidBodyID id1, utility::math::matrix::Transform3D T1, MocapStream::RigidBodyID id2, utility::math::matrix::Transform3D T2);

		void eliminateAndNormalise(std::map<MocapStream::RigidBodyID,float> totalScores);

		std::map<int, bool> sufficientData();

		void compute(const std::map<int, bool>& streamsReady);

		void reset();

		std::vector<std::pair<int,int>> getBestCorrelations(std::vector<utility::math::matrix::Transform3D>* transforms = NULL);
		std::vector<std::pair<int,int>> getRemainingHypotheses();

		float likelihood(float error){
			return std::exp(-error * error);
		}

	};


}
#endif