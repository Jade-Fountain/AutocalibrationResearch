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

		struct SimulationParameters{
			
			struct SinFunc{
				float f = 0;//frequency
				float A = 0;//amplitude
			};

			struct Noise{
				float angle_stddev = 0;
				float disp_stddev = 0;
			};

			struct {
				SinFunc disp;
				SinFunc angle;
			} slip;
			float latency_ms = 0;
			Noise noise;

			SimulationParameters operator+(const SimulationParameters& s){
				SimulationParameters s_;
				
				s_.latency_ms = s.latency_ms + this->latency_ms;
				
				s_.noise.angle_stddev = s.noise.angle_stddev + this->noise.angle_stddev;
				s_.noise.disp_stddev = s.noise.disp_stddev + this->noise.disp_stddev;

				s_.slip.disp.f = s.slip.disp.f + this->slip.disp.f;
				s_.slip.disp.A = s.slip.disp.A + this->slip.disp.A;
				s_.slip.angle.f = s.slip.angle.f + this->slip.angle.f;
				s_.slip.angle.A = s.slip.angle.A + this->slip.angle.A;

				return s_;
			}
			SimulationParameters operator-(const SimulationParameters& s){
				SimulationParameters s_;
				
				s_.latency_ms = this->latency_ms - s.latency_ms ;
				
				s_.noise.angle_stddev = this->noise.angle_stddev - s.noise.angle_stddev ;
				s_.noise.disp_stddev = this->noise.disp_stddev - s.noise.disp_stddev ;

				s_.slip.disp.f = this->slip.disp.f - s.slip.disp.f ;
				s_.slip.disp.A = this->slip.disp.A - s.slip.disp.A ;
				s_.slip.angle.f = this->slip.angle.f - s.slip.angle.f ;
				s_.slip.angle.A = this->slip.angle.A - s.slip.angle.A ;

				return s_;
			}
			SimulationParameters operator*(const float& f){
				SimulationParameters s_;
				
				s_.latency_ms = this->latency_ms * f;
				
				s_.noise.angle_stddev = this->noise.angle_stddev * f;
				s_.noise.disp_stddev = this->noise.disp_stddev * f;

				s_.slip.disp.f = this->slip.disp.f * f;
				s_.slip.disp.A = this->slip.disp.A * f;
				s_.slip.angle.f = this->slip.angle.f * f;
				s_.slip.angle.A = this->slip.angle.A * f;

				return s_;
			}

		};

		std::map<std::pair<int,int>, utility::math::matrix::Transform3D> simWorldTransform;
		std::map<std::pair<int,int>, utility::math::matrix::Transform3D> simLocalTransform;
	private:

		std::map<TimeStamp, Frame> stream;

		std::string stream_name;

		TimeStamp getTimeStamp(const std::chrono::system_clock::time_point& t){
			return std::chrono::duration_cast<std::chrono::microseconds>(t.time_since_epoch()).count();
		}

		Frame createFrame(arma::mat m, bool reflectZ, const std::set<int>& allowedIDs);

		TimeStamp streamStart;

		std::map<MocapStream::RigidBodyID, utility::math::matrix::Transform3D> slippage;

		bool correctForNUbotsCoordinateSystem;

	public:
		//Constructors
		MocapStream() : stream_name(""), slippage(), correctForNUbotsCoordinateSystem(false){}

		MocapStream(std::string name, bool correction) : stream_name(name), correctForNUbotsCoordinateSystem(correction){}

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

		std::map<TimeStamp,Frame>::iterator getUpperBoundIter(const TimeStamp& t);
		std::map<TimeStamp,Frame>::iterator getLowerBoundIter(const TimeStamp& t);
		

		//Heavy functions
		bool loadMocapData(std::string folder_path, const TimeStamp& start_time, const std::chrono::system_clock::time_point& end_time, bool reflectZ = false, const std::set<int>& allowedIDs = std::set<int>());

		bool setRigidBodyInFrame(const std::chrono::system_clock::time_point& frame_time, const unsigned int& id, const utility::math::matrix::Transform3D& pose);
		
		bool setRigidBodyInFrame(const TimeStamp& frame_time, const unsigned int& id, const utility::math::matrix::Transform3D& pose);

		std::map<MocapStream::RigidBodyID, arma::vec> getInvariates(TimeStamp now);
		
		std::map<MocapStream::RigidBodyID, arma::vec> getStates(TimeStamp now);
		
		std::map<MocapStream::RigidBodyID, utility::math::matrix::Transform3D> getCompleteStates(TimeStamp now);

		std::map<MocapStream::RigidBodyID, arma::vec> getSimulatedStates(TimeStamp now, std::vector<RigidBodyID> ids);
		
		std::map<MocapStream::RigidBodyID, utility::math::matrix::Transform3D> getCompleteSimulatedStates(TimeStamp now, std::vector<RigidBodyID> ids, const SimulationParameters& sim);

	};

}
#endif