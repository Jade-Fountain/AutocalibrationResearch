#include "SensorCorrelator.h"

#include "message/support/Configuration.h"
#include "message/fusion/Fusion.h"

namespace module {
namespace fusion {

    using message::support::Configuration;
    using message::input::OpenNIData;
    using message::input::RigidBodyFrame;
    
    using message::fusion::MatchResults;

    using autocal::MocapStream;

    SensorCorrelator::SensorCorrelator(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("SensorCorrelator.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file SensorCorrelator.yaml
        });

        on<Startup>().then([this]{
		    //Optional simulation parameters
		    autocal::MocapStream openniStream(OPENNI_STREAM);
		    autocal::MocapStream mocapStream(MOCAP_STREAM, true);

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

    		//Compute timestamp
	        std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();    
	        double frame_time_since_start = std::chrono::duration_cast<std::chrono::milliseconds>(now-start_time).count() / float(std::milli::den);  

	        autocal::TimeStamp current_timestamp = std::chrono::duration_cast<std::chrono::microseconds>(now-start_time).count();
    		
	        //Add openni data
    		if(openniData){
    			for(auto& user : openniData->users){
					// log("OpenNIData Data Received for User ", user.first, " : ", user.second.poses.size(), "rigid bodies");
    				addData(OPENNI_STREAM, user.second, current_timestamp, user.first);

    			}
    		}
			
			//Add mocap data
			if(mocapData){
				// log("Mocap Data Received :", mocapData->poses.size(), "rigid bodies");
				addData(MOCAP_STREAM, *mocapData, current_timestamp);
    		}

    		//Init message:
    		auto results = std::make_unique<MatchResults>();
    		results->stream1 = OPENNI_STREAM;
    		results->stream2 = MOCAP_STREAM;
    		//Match streams
	        results->matches = sensorPlant.matchStreams(OPENNI_STREAM, MOCAP_STREAM, current_timestamp, 0);

	        //Debug:
	        for(auto& match : results->matches){
	        	std::cout << "Matched: " << match.first << " with " << match.second << std::endl;
	        }
	        //Send to visualiser
	        emit(results);
    	});
    }

    void SensorCorrelator::addData(std::string stream, const RigidBodyFrame& rigidBodies, autocal::TimeStamp t, int id_offset){
		for(auto& pose : rigidBodies.poses){
			// std::cout << stream << " pose " << pose.second << std::endl;
			sensorPlant.mocapRecording.addMeasurement(stream, t, pose.first + id_offset, pose.second);
		}
    }
}
}
