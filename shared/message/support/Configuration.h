#ifndef EXTENSION_CONFIGURATION_H
#define EXTENSION_CONFIGURATION_H

#include <cstdlib>
#include <nuclear>
#include <yaml-cpp/yaml.h>

#include "FileWatch.h"
#include "utility/strutil/strutil.h"

namespace message {
namespace support {

    /**
     * TODO document
     *
     * @author Trent Houliston
     */
    struct Configuration {
        std::string path;
        YAML::Node config;

        Configuration() : path(""), config() {};
        Configuration(const std::string& path, YAML::Node config) : path(path), config(config) {};

        Configuration operator [] (const std::string& key) {
            return Configuration(path, config[key]);
        }

        const Configuration operator [] (const std::string& key) const {
            return Configuration(path, config[key]);
        }

        Configuration operator [] (const char* key) {
            return Configuration(path, config[key]);
        }

        const Configuration operator [] (const char* key) const {
            return Configuration(path, config[key]);
        }

        Configuration operator [] (size_t index) {
            return Configuration(path, config[index]);
        }

        const Configuration operator [] (size_t index) const {
            return Configuration(path, config[index]);
        }

        template <typename T>
        T as() const {
            return config.as<T>();
        }

        // All of these disables for this template are because the std::string constructor is magic and screwy
        template <
                typename T
                , typename Decayed = typename std::decay<T>::type
                , typename = typename std::enable_if<
                        !std::is_same<
                                const char*
                                , Decayed
                        >::value
                        && !std::is_same<
                                std::allocator<char>
                                , Decayed
                        >::value
                        && !std::is_same<
                                std::initializer_list<char>
                                , Decayed
                        >::value
                        && !std::is_same<
                                char
                                , Decayed
                        >::value
                >::type
        >
        operator T() const {
            return config.as<T>();
        }

        // The conversion for string is fully specialised because strings get screwy
        // because of their auto conversion to const char* etc
        operator std::string() const {
            return config.as<std::string>();
        }
    };

}  // support
}  // message

// NUClear configuration extension
namespace NUClear {
    namespace dsl {
        namespace operation {
            template <>
            struct DSLProxy<::message::support::Configuration> {

                template <typename DSL, typename TFunc>
                static inline threading::ReactionHandle bind(Reactor& reactor, const std::string& label, TFunc&& callback, const std::string& path) {
                    return DSLProxy<::message::support::FileWatch>::bind<DSL>(reactor, label, callback, "config/" + path,
                                                                         ::message::support::FileWatch::ATTRIBUTE_MODIFIED
                                                                       | ::message::support::FileWatch::CREATED
                                                                       | ::message::support::FileWatch::UPDATED
                                                                       | ::message::support::FileWatch::MOVED_TO);
                }

                template <typename DSL>
                static inline std::shared_ptr<::message::support::Configuration> get(threading::Reaction& t) {

                    // Get the file watch event
                    ::message::support::FileWatch watch = DSLProxy<::message::support::FileWatch>::get<DSL>(t);

                    // Check if the watch is valid
                    if(watch && utility::strutil::endsWith(watch.path, ".yaml")) {
                        // Return our yaml file
                        try {
                            return std::make_shared<::message::support::Configuration>(watch.path, YAML::LoadFile(watch.path));
                        } catch (const YAML::ParserException& e){
                            throw std::runtime_error(watch.path + " " + std::string(e.what()));
                        }
                    }
                    else {
                        // Return an empty configuration (which will show up invalid)
                        return std::shared_ptr<::message::support::Configuration>(nullptr);
                    }
                }
            };
        }

        // Configuration is transient
        namespace trait {
            template <>
            struct is_transient<std::shared_ptr<::message::support::Configuration>> : public std::true_type {};
        }
    }
}

#endif //EXTENSION_CONFIGURATION_H
