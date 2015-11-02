#ifdef __APPLE__
    // #include <OpenGL/gl.h>
    #include <GL/glew.h>  

    #include <GLUT/glut.h>
#else
  #include <GL/gl.h>
  #include <GL/glut.h>
#endif 

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

  
static bool setUpGLEW()  
{  
    //Initialize GLEW  
    glewExperimental = GL_TRUE;
    GLenum err = glewInit();  
  
    //If GLEW hasn't initialized  
    if (err != GLEW_OK)   
    {  
        std::cout << "GLEW Error: " << glewGetErrorString(err) <<  std::endl;  
        return false;  
    }

    return true;
}
 
static void drawBasis(float scale){
    GLUquadricObj *quadratic;
    quadratic = gluNewQuadric();
    float coneRadius = 0.3 * scale;
    float coneHeight = 0.6 * scale;
    int numberOfConeSegments = 10;
    
    glDisable(GL_TEXTURE_2D);
    
    glEnable(GL_LIGHTING);
    glLightf(GL_LIGHT0, GL_CONSTANT_ATTENUATION, 0.0f);
    glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, 0.0f);
    glLightf(GL_LIGHT0, GL_QUADRATIC_ATTENUATION, 0.0f);
    GLfloat diff[4] = {1.0, 1.0, 1.0, 1.0};
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diff);    
    glLightfv(GL_LIGHT0, GL_AMBIENT, diff);
    glLightfv(GL_LIGHT0, GL_SPECULAR, diff);
    //mat x
    GLfloat x_diff[4] = {1.0, 0.0, 0.0, 1.0};
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, x_diff);
    //cylinder x
    glRotatef(90,0,1,0);
    gluCylinder(quadratic,coneRadius / 2, coneRadius / 2, scale, numberOfConeSegments,1);
    glRotatef(-90,0,1,0);
    //cone x
    glTranslatef(scale,0,0);
    glRotatef(90,0,1,0);
    glutSolidCone(coneRadius, coneHeight, numberOfConeSegments, 1);
    glRotatef(-90,0,1,0);
    glTranslatef(-scale,0,0);

    //Material y
    GLfloat y_diff[4] = {0.0, 1.0, 0.0, 1.0};
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, y_diff);
    
    //cylinder y
    glRotatef(-90,1,0,0);
    gluCylinder(quadratic,coneRadius / 2, coneRadius / 2, scale, numberOfConeSegments,1);
    glRotatef(90,1,0,0);

    //cone y
    glTranslatef(0,scale,0);
    glRotatef(-90,1,0,0);
    glutSolidCone(coneRadius, coneHeight, numberOfConeSegments, 1);
    glRotatef(90,1,0,0);
    glTranslatef(0,-scale,0);
    
    //mat z
    GLfloat z_diff[4] = {0.0, 0.0, 1.0, 1.0};
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, z_diff);
    //cylinder z
    gluCylinder(quadratic,coneRadius / 2, coneRadius / 2, scale, numberOfConeSegments,1);
    
    //cone z
    glTranslatef(0,0,scale);
    glutSolidCone(coneRadius, coneHeight, numberOfConeSegments, 1);
    glDisable(GL_LIGHTING);
    delete quadratic;
}

static void drawCrossHair(){
    float w = 0.03; //crosshair width
    float g = w / 4.0; //crosshair gap


    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glLineWidth(2.5); 
    glColor3f(1.0, 0.0, 0.0);
    glBegin(GL_LINES);

    //Diag
    glVertex2f(w, w);
    glVertex2f(-w, -w);

    glVertex2f(-w, w);
    glVertex2f(w, -w);
    
    //Top right
    glVertex2f(w, w);
    glVertex2f(w, g);
    
    glVertex2f(g, w);
    glVertex2f(w, w);
    
    //bottom right
    glVertex2f(w, -w);
    glVertex2f(w, -g);
    
    glVertex2f(g, -w);
    glVertex2f(w, -w);

    //Bottom left
    glVertex2f(-w, -w);
    glVertex2f(-w, -g);
    
    glVertex2f(-g, -w);
    glVertex2f(-w, -w);

    //top left
    glVertex2f(-w, w);
    glVertex2f(-w, g);
    
    glVertex2f(-g, w);
    glVertex2f(-w, w);
 

    glEnd();
}

