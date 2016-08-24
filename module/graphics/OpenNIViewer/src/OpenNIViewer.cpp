#include "OpenNIViewer.h"

#include "extension/Configuration.h"
#include "message/input/OpenNIImage.h"

namespace module {
namespace graphics {

    using extension::Configuration;
    using message::input::OpenNIImage;

    //Define the key input callback  
	void OpenNIViewer::handleInput(const sf::Window& w, double time_since_start){
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Escape))
		{
		    std::cout << "Exiting .." << std::endl;
		    running = false;
		}
	}

    OpenNIViewer::OpenNIViewer(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)),
    window(sf::VideoMode(640*2, 480*2), "OpenGL", sf::Style::Default, sf::ContextSettings(32)) {

        on<Configuration>("OpenNIViewer.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file OpenNIViewer.yaml
        });

        on<Startup>.then([this]{
		    start = std::chrono::steady_clock::now();  
        });

        on<Trigger<std::vector<openni::RGB888Pixel>>, Single, MainThread>().then("Main Loop",
        	[this](const OpenNIImage& image){
	       	//Get current time
	        auto now = std::chrono::steady_clock::now();    
	        double frame_time_since_start = std::chrono::duration_cast<std::chrono::milliseconds>(now-start).count() / float(std::milli::den);  
	        
	        //Compute timestamp in microseconds
	        autocal::TimeStamp current_timestamp = videoStartTime + std::chrono::duration_cast<std::chrono::microseconds>(now-start).count();
	        
	        //Set window active
	        window.setActive(true);

	        //Check input
	        handleInput(window, frame_time_since_start);


	        //Clear color buffer and enable textures
	        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	        glEnable(GL_TEXTURE_2D);

	        //Draw visuals to screen
	        glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP_SGIS, GL_TRUE);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image.width, image.height, 0, GL_RGB, GL_UNSIGNED_BYTE, image.data());
			// glTexImage2D(GL_TEXTURE_2D, 0, GL_R, depthFrame.getVideoMode().getResolutionX(), depthFrame.getVideoMode().getResolutionY(), 0, GL_R, GL_UNSIGNED_BYTE, depthFrame.getData());

			// Display the OpenGL texture map
			glColor4f(1,1,1,1);

			glEnable(GL_TEXTURE_2D);
			glBegin(GL_QUADS);

			// upper left
			glTexCoord2f(0, 0);
			glVertex2f(-1, 1);
			// upper right
			glTexCoord2f(1, 0);
			glVertex2f(1, 1);
			// bottom right
			glTexCoord2f(1, 1);
			glVertex2f(1, -1);
			// bottom left
			glTexCoord2f(0, 1);
			glVertex2f(-1, -1);

			glEnd();
			glDisable(GL_TEXTURE_2D);

	        window.display();

	        //Shutdown if killed
		    if(!running){
		    	powerplant.shutdown();
		    }

	        // check all the window's events that were triggered since the last iteration of the loop
	        sf::Event event;
	        while (window.pollEvent(event))
	        {
	            // "close requested" event: we close the window
	            if (event.type == sf::Event::Closed)
	                powerplant.shutdown();
	        }

        });
    }	
}
}
