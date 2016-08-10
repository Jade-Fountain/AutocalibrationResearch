#ifndef MODULES_EXTENSION_FILEWATCHER_H
#define MODULES_EXTENSION_FILEWATCHER_H

#include <nuclear>
#include <libfswatch/c++/monitor.hpp>

namespace module {
namespace support {
namespace extension {

    // This is the callback that is used to handle file system events
    void fswatchCallback(const std::vector<fsw::event>& events, void* reactor);

    class FileWatcher : public NUClear::Reactor {
    private:
        std::map<std::string, std::map<std::string, std::vector<std::pair<std::shared_ptr<NUClear::threading::Reaction>, int>>>> handlers;
        std::mutex runMutex;
        std::unique_ptr<fsw::monitor> monitor;
    public:
        /// @brief Called by the powerplant to build and setup the FileWatcher reactor.
        explicit FileWatcher(std::unique_ptr<NUClear::Environment> environment);

        friend void fswatchCallback(const std::vector<fsw::event>& events, void* reactor);
    };

}  // extension
}  // support
}  // module

#endif  // MODULES_EXTENSION_FILEWATCHER_H
