#include "SensorCorrelator.h"

#include "message/support/Configuration.h"

namespace module {
namespace fusion {

    using message::support::Configuration;
    using message::input::OpenNIData;
    using message::input::RigidBodyFrame;

    using autocal::MocapStream;

    SensorCorrelator::SensorCorrelator(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("SensorCorrelator.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file SensorCorrelator.yaml
        });

        on<Startup>().then([this]{
		    //Optional simulation parameters
		    autocal::MocapStream openniStream(OPENNI_STREAM);
		    autocal::MocapStream mocapStream(MOCAP_STREAM);

		    sensorPlant.addStream(openniStream);
		    sensorPlant.addStream(mocapStream);
	       
	        start_time = std::chrono::system_clock::now();    
        });

        on<Every<30, 
        Per<std::chrono::seconds>>,
        Optional<With<OpenNIData>>,
        Optional<With<RigidBodyFrame>>,
        Single>().then("SensorCorrelator - Sensor Matching Loop",
    	[this](
    		const std::shared_ptr<const OpenNIData> openniData,
    		const std::shared_ptr<const RigidBodyFrame> mocapData
    		){

	        std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();    
	        double frame_time_since_start = std::chrono::duration_cast<std::chrono::milliseconds>(now-start_time).count() / float(std::milli::den);  

	        autocal::TimeStamp current_timestamp = std::chrono::duration_cast<std::chrono::microseconds>(now-start_time).count();
    		

    		if(openniData){
    			for(auto& user : openniData->users){
    				addData(OPENNI_STREAM, user.second, current_timestamp, user.first);
    			}
    		}
			
			if(mocapData){
				addData(MOCAP_STREAM, *mocapData, current_timestamp);
    		}

    	});
    }

    void SensorCorrelator::addData(std::string stream, const RigidBodyFrame& rigidBodies, autocal::TimeStamp t, int id_offset){
		for(auto& pose : rigidBodies.poses){
			sensorPlant.mocapRecording.addMeasurement(stream, t, pose.first, pose.second);
		}
    }
}
}
