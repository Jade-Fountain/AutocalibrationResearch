#ifdef __APPLE__
    // #include <OpenGL/gl.h>
    #include <GL/glew.h>  

    #include <GLFW/glfw3.h>  
    #include <GLUT/glut.h>
#else
  #include <GL/gl.h>
  #include <GL/glut.h>
#endif 

//Define an error callback  
static void error_callback(int error, const char* description)  
{  
    fputs(description, stderr);  
    getchar();  
}  
  
//Define the key input callback  
static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)  
{  
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)  
    glfwSetWindowShouldClose(window, GL_TRUE);  
   
} 

static void cursor_position_callback(GLFWwindow* window, double xpos, double ypos)
{
}

static void handle_input(GLFWwindow* window, double time_since_start){

}

static void checkGLError(){

    GLenum error;
    while((error = glGetError()) != GL_NO_ERROR){  
        std::cout << "GL Error = " << std::endl;  
        switch(error)
        {
            case 0:
                std::cout <<"Internal error in glGetError()" << std::endl;
                break;
            
            case GL_INVALID_ENUM:
                std::cout <<"Invalid enum" << std::endl;
                break;
            
            case GL_INVALID_VALUE:
                std::cout <<"Invalid value" << std::endl;
                break;
            
            case GL_INVALID_OPERATION:
                std::cout <<"Invalid operation" << std::endl;
                break;
            
            case GL_STACK_OVERFLOW:
                std::cout <<"Stack overflow" << std::endl;
                break;
            
            case GL_STACK_UNDERFLOW:
                std::cout <<"Stack underflow" << std::endl;
                break;
            
            case GL_OUT_OF_MEMORY:
                std::cout <<"Out of memory" << std::endl;
                break;
            
            case GL_TABLE_TOO_LARGE:
                std::cout <<"Table too large" << std::endl;
                break;
            
            default:
                std::cout << "Unknown error "<< error << std::endl;
        }
    }
    
}

  
static GLFWwindow* setUpGLWindow(int w, int h)  
{  
    //Set the error callback  
    glfwSetErrorCallback(error_callback);  
  
    //Initialize GLFW  
    if (!glfwInit())  
    {  
        exit(EXIT_FAILURE);  
    }  
  
    //Set the GLFW window creation hints - these are optional  
    //glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3); //Request a specific OpenGL version  
    //glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2); //Request a specific OpenGL version  
    //glfwWindowHint(GLFW_SAMPLES, 4); //Request 4x antialiasing  
    //glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  
    GLFWwindow* window;  
    
    //Create a window and create its OpenGL context  
    window = glfwCreateWindow(640, 480, "Test Window", NULL, NULL);  
  
    //If the window couldn't be created  
    if (!window)  
    {  
        fprintf( stderr, "Failed to open GLFW window.\n" );  
        glfwTerminate();  
        exit(EXIT_FAILURE);  
    }  
  
    //This function makes the context of the specified window current on the calling thread.   
    glfwMakeContextCurrent(window);  
  
    //Sets the key callback  
    glfwSetKeyCallback(window, key_callback);  
  
    //Initialize GLEW  
    glewExperimental = GL_TRUE;
    GLenum err = glewInit();  
  
    //If GLEW hasn't initialized  
    if (err != GLEW_OK)   
    {  
        fprintf(stderr, "Error: %s\n", glewGetErrorString(err));  
        return nullptr;  
    }

    //Set a background color  
    glClearColor(0.0f, 0.0f, 1.0f, 0.0f);  
    glEnable(GL_LIGHT0);
    glEnable(GL_DEPTH_TEST);

    return window;
}
  
static void destroyGLWindow(GLFWwindow* window){
  
    //Close OpenGL window and terminate GLFW  
    glfwDestroyWindow(window);  
    //Finalize and clean up GLFW  
    glfwTerminate();  
  
    exit(EXIT_SUCCESS);
}