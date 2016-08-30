#ifndef MODULES_SUPPORT_LOGGING_CONSOLELOGHANDLER_H
#define MODULES_SUPPORT_LOGGING_CONSOLELOGHANDLER_H

#include <nuclear>
#include <mutex>

namespace module {
    namespace support {
        namespace logging {

            /**
             * Handles the logging of log messages to the console in a thread safe manner.
             *
             * @author Jake Woods
             */
            class ConsoleLogHandler : public NUClear::Reactor {
            private:
                std::mutex mutex;
            public:
                explicit ConsoleLogHandler(std::unique_ptr<NUClear::Environment> environment);
            };

        }  // logging
    }  // support
}  // modules

#endif  // MODULES_SUPPORT_LOGGING_CONSOLELOGHANDLER_H
