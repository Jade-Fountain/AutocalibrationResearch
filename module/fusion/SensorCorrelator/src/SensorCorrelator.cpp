#include "SensorCorrelator.h"

#include "message/support/Configuration.h"
#include "message/input/MotionCapture.h"

namespace module {
namespace fusion {

    using message::support::Configuration;
    using message::input::OpenNIData;

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
        });

        on<Every<30, 
        Per<std::chrono::seconds>>,
        Optional<With<OpenNIData>>,
        Single>().then("SensorCorrelator - Sensor Matching Loop",
    	[this](const std::shared_ptr<const OpenNIData> openniData){



    	});
    }
}
}
