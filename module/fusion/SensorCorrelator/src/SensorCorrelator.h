#ifndef MODULE_FUSION_SENSORCORRELATOR_H
#define MODULE_FUSION_SENSORCORRELATOR_H

#include <nuclear>
#include "utility/autocal/SensorPlant.h"
#include "message/input/MotionCapture.h"


namespace module {
namespace fusion {

    class SensorCorrelator : public NUClear::Reactor {
		autocal::SensorPlant sensorPlant;

		const std::string OPENNI_STREAM = "openni";
		const std::string MOCAP_STREAM = "mocap";
	    
	    std::chrono::time_point<std::chrono::system_clock> start_time;    

    	void addData(std::string stream, const message::input::RigidBodyFrame& rigidBodies, autocal::TimeStamp t, int id_offset = 0);
    public:
        /// @brief Called by the powerplant to build and setup the SensorCorrelator reactor.
        explicit SensorCorrelator(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULE_FUSION_SENSORCORRELATOR_H
