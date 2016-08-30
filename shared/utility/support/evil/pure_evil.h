#ifndef UTILITY_SUPPORT_EVIL_PUREEVIL_H
#define UTILITY_SUPPORT_EVIL_PUREEVIL_H

#include <vector>
#include <string>

// If we are not in debug mode, don't build it! Please don't build it!
#if !defined(NDEBUG) && !defined(__APPLE__)

namespace utility {
    namespace support {
        namespace evil {

            struct StackFrame {

                StackFrame()
                : pc()
                , file()
                , lineno()
                , function() {}

                StackFrame(uintptr_t pc, std::string file, int lineno, std::string function)
                : pc(pc)
                , file(file)
                , lineno(lineno)
                , function(function) {}

                uintptr_t pc;
                std::string file;
                int lineno;
                std::string function;
            };

            thread_local std::vector<StackFrame> stack;
            thread_local std::string exception_name;
        }
    }
}

#endif  // NDEBUG or APPLE

#endif  // UTILITY_SUPPORT_EVIL_PUREEVIL_H
