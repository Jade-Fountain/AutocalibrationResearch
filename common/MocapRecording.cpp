/*
author Jake Fountain
This code is part of mocap-kinect experiments*/

#include "MocapRecording.h"

namespace autocal {

	using utility::math::matrix::Transform3D;
	using utility::math::matrix::Rotation3D;
	using utility::math::geometry::UnitQuaternion;

	void MocapRecording::addMeasurement(const std::string& name, 
									 const TimeStamp& timeStamp, 
									 const MocapStream::RigidBodyID& rigidBodyId, 
									 const Transform3D& pose)
	{	
		//Create a new stream if one with this name doesnt exist
		if(streams.count(name) == 0){
			std::cout << "Initialising mocap stream: " << name << std::endl;
			streams[name] = MocapStream(name);
		}
		//Set the data
		streams[name].setRigidBodyInFrame(timeStamp, rigidBodyId, pose);
	}

	void MocapRecording::markStartOfStreams(TimeStamp now){
		for(auto& stream : streams){
			stream.second.markStart(now);
		}
	}




}