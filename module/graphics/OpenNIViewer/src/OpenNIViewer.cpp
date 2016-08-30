#include "OpenNIViewer.h"

#include "message/support/Configuration.h"
#include "message/input/OpenNIImage.h"
#include "message/input/MotionCapture.h"

#include "utility/autocal/GraphicsTools.h"
#include "utility/math/matrix/Transform3D.h"

namespace module {
namespace graphics {

    using utility::math::matrix::Transform3D;
    using message::input::OpenNIData;

    using message::support::Configuration;
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

        on<Startup>().then([this]{
		    start = std::chrono::steady_clock::now();  

		    proj =

		        // glm::perspectiveFov<float>(PSEYE_FOV_BLUE_DOT,
		        // image->width, image->height, 0.01f, 10.0f);

		        glm::perspective(   float(49.3 * 3.14159 / 180.0),            //VERTICAL FOV
		                            float(window.getSize().x) / float(window.getSize().y),  //aspect ratio
		                            0.1f,         //near plane distance (min z)
		                            1000.0f           //Far plane distance (max z)
		                            );

		        		    //GLEW
		    bool success = setUpOpenGL();
	  	    if(!success){
	  	    	std::cout << "OpenGL Setup Failed! Shutting down" << std::endl; 
			    powerplant.shutdown();
			}

		    //Check errors
		    checkGLError();
        });

        on<Trigger<OpenNIImage>, Optional<With<OpenNIData>>, Single, MainThread>().then("Main Loop",
        	[this](const OpenNIImage& image, 
    			   const std::shared_ptr<const OpenNIData> openniData
    			   //TODO:
    			   // const std::shared_ptr<const MatchData> matchData
    			   ){
	       	//Get current time
	        auto now = std::chrono::steady_clock::now();    
	        double frame_time_since_start = std::chrono::duration_cast<std::chrono::milliseconds>(now-start).count() / float(std::milli::den);  
	        	        
	        //Set window active
	        window.setActive(true);

	        //Check input
	        handleInput(window, frame_time_since_start);


	        glMatrixMode(GL_PROJECTION);
	        glLoadIdentity();
	        glMatrixMode(GL_MODELVIEW);
	        glLoadIdentity();

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
	        
	        //Draw tracking data

	        // // Clear depth buffer  
	        glClear(GL_DEPTH_BUFFER_BIT);

	        glMatrixMode(GL_PROJECTION);
		    glLoadMatrixf(glm::value_ptr(proj));

			float basisScale = 0.1;
	        //Draw mocap rigid bodies
	        if(openniData){
		        for(auto& user : openniData->users){
			        for(auto& rigidBody : user.second.poses){
			        	//Extract ID
			        	int id = rigidBody.first;
			        	
			        	//4x4 matrix pose
			            Transform3D pose = rigidBody.second;
			            
			            //Convert to m from mm
			            pose.translation() = pose.translation() * 0.001;
			            
			            // pose = Transform3D::createRotationY(M_PI) * pose;
			            pose = Transform3D::createScale(arma::vec3{1,1,-1}) * pose;
			            
			            //Load pose into opengl as view matrix
			            glMatrixMode(GL_MODELVIEW);
			            glLoadMatrixd(pose.memptr());
			            
			            drawBasis(basisScale);
			        }
		        }	
	        } else {
	        	std::cout << "NO Kinect RECEIVED" << std::endl;
	        }


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