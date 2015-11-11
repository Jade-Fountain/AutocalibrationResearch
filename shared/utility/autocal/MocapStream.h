/*
author Jake Fountain
This code is part of mocap-kinect experiments*/
#include <armadillo>
#include <chrono>
#include <dirent.h>
#include <map>
#include <set>
#include "utility/math/matrix/Transform3D.h"
#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/geometry/UnitQuaternion.h"

#ifndef AUTOCAL_MOCAP_STREAM
#define AUTOCAL_MOCAP_STREAM

namespace autocal {
	
	typedef long long int TimeStamp;

	class MocapStream {
	public:
		typedef unsigned int RigidBodyID;

		//TODO generalise to other sensors, not just rigid bodies
		struct RigidBody {
			utility::math::matrix::Transform3D pose;
		};

		struct Frame {
			std::map<RigidBodyID, RigidBody> rigidBodies;

			std::string toString();

			static Frame interpolate(const Frame& A, const Frame& B, float alpha){
				Frame interp;
				for(const auto& rb : A.rigidBodies){
					const RigidBodyID& idA = rb.first;
					const utility::math::matrix::Transform3D& poseA = rb.second.pose;
					if(B.rigidBodies.count(idA) != 0){
						const utility::math::matrix::Transform3D& poseB = B.rigidBodies.at(idA).pose;
						interp.rigidBodies[idA] = RigidBody();
						interp.rigidBodies[idA].pose = utility::math::matrix::Transform3D::interpolate(poseA, poseB, alpha);
					}
				}
				return interp;
			}

		};

	private:

		std::map<TimeStamp, Frame> stream;

		std::string stream_name;

		TimeStamp getTimeStamp(const std::chrono::system_clock::time_point& t){
			return std::chrono::duration_cast<std::chrono::microseconds>(t.time_since_epoch()).count();
		}

		Frame createFrame(arma::mat m, bool reflectZ, const std::set<int>& allowedIDs);

		TimeStamp streamStart;

		bool correctForAutocalibrationCoordinateSystem;

	public:
		//Constructors
		MocapStream() : stream_name(""), correctForAutocalibrationCoordinateSystem(false){}

		MocapStream(std::string name, bool correction) : stream_name(name), correctForAutocalibrationCoordinateSystem(correction){}

		//Accessors and small utilities
		void markStart(TimeStamp t){
			streamStart = t;
		}

		int size() const {return stream.size();}

		bool isEmpty() const {return stream.empty();}
		
		std::string name() const {return stream_name;}

		std::string toString();

		std::map<TimeStamp, Frame>& frameList(){return stream;}
		
		std::map<TimeStamp,Frame>::iterator begin(){return stream.begin();}
		std::map<TimeStamp,Frame>::iterator end(){return stream.end();}
		
		//Frame retrieval
		Frame getFrame(const std::chrono::system_clock::time_point& t);
		Frame getInterpolatedFrame(const TimeStamp& t);
		Frame getFrame(const TimeStamp& t);
		TimeStamp getFrameTime(const TimeStamp& t);

		//Iterator accessor
		std::map<TimeStamp,Frame>::iterator getUpperBoundIter(const TimeStamp& t);
		std::map<TimeStamp,Frame>::iterator getLowerBoundIter(const TimeStamp& t);
		
		//Load data from files
		bool loadMocapData(std::string folder_path, const TimeStamp& start_time, const std::chrono::system_clock::time_point& end_time, bool reflectZ = false, const std::set<int>& allowedIDs = std::set<int>());

		//set data using different time indicators
		bool setRigidBodyInFrame(const std::chrono::system_clock::time_point& frame_time, const unsigned int& id, const utility::math::matrix::Transform3D& pose, bool correctCoordinateSystem, bool reflectZAxis);
		bool setRigidBodyInFrame(const TimeStamp& frame_time, const unsigned int& id, const utility::math::matrix::Transform3D& pose, bool correctCoordinateSystem, bool reflectZAxis);
		
		//Get the latest poses of the recorded data
		std::map<MocapStream::RigidBodyID, utility::math::matrix::Transform3D> getCompleteStates(TimeStamp now);
		
	};

}
#endif