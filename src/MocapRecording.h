/*
author Jake Fountain
This code is part of mocap-kinect experiments*/
#include <armadillo>
#include <chrono>
#include <dirent.h>
#include <map>
#include "MocapStream.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/geometry/UnitQuaternion.h"

#ifndef AUTOCAL_MOCAP_RECORDING
#define AUTOCAL_MOCAP_RECORDING

namespace autocal {

	class MocapRecording {

		std::map<std::string, MocapStream> streams;
	
	public:
	
		MocapStream& getStream(std::string stream_name){
			return streams[stream_name];
		}

		bool streamPresent(std::string stream_name){
			return streams.count(stream_name) != 0;
		}

		void addMeasurement(const std::string& name, const TimeStamp& timeStamp, const MocapStream::RigidBodyID& rigidBodyId, const utility::math::matrix::Transform3D& pose);
		
		void markStartOfStreams(TimeStamp now);


	};


}
#endif