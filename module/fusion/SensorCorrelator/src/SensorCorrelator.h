#ifndef MODULE_FUSION_SENSORCORRELATOR_H
#define MODULE_FUSION_SENSORCORRELATOR_H

#include <nuclear>
#include "utility/autocal/SensorPlant.h"

namespace module {
namespace fusion {

    class SensorCorrelator : public NUClear::Reactor {
		autocal::SensorPlant sensorPlant;

		const std::string OPENNI_STREAM = "openni";
		const std::string MOCAP_STREAM = "mocap";

    public:
        /// @brief Called by the powerplant to build and setup the SensorCorrelator reactor.
        explicit SensorCorrelator(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULE_FUSION_SENSORCORRELATOR_H
