#ifndef MODULE_GRAPHICS_OPENNIVIEWER_H
#define MODULE_GRAPHICS_OPENNIVIEWER_H

#include <nuclear>

#ifdef __APPLE__
    #include <GL/glew.h>  

    #include <GLFW/glfw3.h>  
    #include <GLUT/glut.h>
#else
  #include <GL/gl.h>
  #include <GL/glut.h>
#endif 

#include "opencv2/opencv.hpp"
#include "SFML/Window.hpp"

namespace module {
namespace graphics {

    class OpenNIViewer : public NUClear::Reactor {
		sf::Window window;

    public:
        /// @brief Called by the powerplant to build and setup the OpenNIViewer reactor.
        explicit OpenNIViewer(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULE_GRAPHICS_OPENNIVIEWER_H
