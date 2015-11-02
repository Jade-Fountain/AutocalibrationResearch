/*
 * This file is part of Autocalibration Codebase.
 *
 * The Autocalibration Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The Autocalibration Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the Autocalibration Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2015 Autocalibration <nubots@nubots.net>
 */

#ifndef MODULES_SUPPORT_EXTENSION_FILEWATCHER_H
#define MODULES_SUPPORT_EXTENSION_FILEWATCHER_H

#include <nuclear>

namespace modules {
namespace support {
namespace extension {

    class FileWatcher : public NUClear::Reactor {
    private:
        int watcherFd;
        std::map<std::string, std::map<std::string, std::vector<std::pair<std::shared_ptr<NUClear::threading::Reaction>, int>>>> handlers;
        std::map<int, std::string> watchPaths;

    public:
        /// @brief Called by the powerplant to build and setup the FileWatcher reactor.
        explicit FileWatcher(std::unique_ptr<NUClear::Environment> environment);
    };

}
}
}

#endif  // MODULES_SUPPORT_EXTENSION_FILEWATCHER_H
