#include "parsing.h"
#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#define GL_GLEXT_PROTOTYPES 1

#include <math.h>
#define _USE_MATH_DEFINES

using namespace std;

///////////////////////////////////////////////////////////////////////////////////////////////////

/* The following are function prototypes for the functions that you will most
 * often write when working in OpenGL.
 *
 * Details on the functions will be given in their respective implementations
 * further below.
 */

void init(void);
void reshape(int width, int height);
void display(void);

void init_lights();
void set_lights();
void draw_objects();

void mouse_pressed(int button, int state, int x, int y);
void mouse_moved(int x, int y);
void key_pressed(unsigned char key, int x, int y);
Quat getCurrentRotation();
Quat computeRotationQuat();
Vector3d convertScreenToNDC(int px, int py);

///////////////////////////////////////////////////////////////////////////////////////////////////

/* The following are the typical camera specifications and parameters.
 */

Camera c;
Perspective p;

///////////////////////////////////////////////////////////////////////////////////////////////////

/* Self-explanatory lists of lights and objects.
 */
vector<Point_Light> lights;
vector<Object> objects;
int num_lights;

/* Keeps track of quaternions for Arcball
 */
Quat last_rotation;
Quat current_rotation;

/* Keeps track of the screen resolution */
int xres, yres;

/* Constants related to shading
 */
GLenum shaderProgram;
string vertProgFileName, fragProgFileName;

///////////////////////////////////////////////////////////////////////////////////////////////////

/* The starting position of the mouse */
int spx, spy;
/* The current position of the mouse */
int cpx, cpy;
float mouse_scale_x, mouse_scale_y;

const float step_size = 0.2;
float x_view_angle = 0, y_view_angle = 0;

bool is_pressed = false;
bool wireframe_mode = false;

///////////////////////////////////////////////////////////////////////////////////////////////////

/* From here on are all the function implementations.
 */

/* 'init' function:
 * 
 * As you would expect, the 'init' function initializes and sets up the
 * program. It should always be called before anything else.
 *
 */
void init(void)
{
    /* Use backface culling
     */
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    /* Use depth buffering
     */
    glEnable(GL_DEPTH_TEST);

    /* Automatically normalize our normal vectors before it passes them 
     * into the normal arrays discussed below.
     */
    glEnable(GL_NORMALIZE);

    /* The following two lines tell OpenGL to enable its "vertex array" and
     * "normal array" functionality. 
     */
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    glFrustum(p.left_param, p.right_param,
              p.bottom_param, p.top_param,
              p.near_param, p.far_param);
    glMatrixMode(GL_MODELVIEW);

    /* The next line calls our function that tells OpenGL to initialize some
     * lights to represent our Point Light structs. 
     */
    init_lights();
     /* Initialize quaternions for the arcball mechanism
     */
    last_rotation = Quat();
    current_rotation = Quat();
    GLenum err = glewInit();
}

/* 
 * The 'reshape' function is supposed to tell your program how to react
 * whenever the program window is resized. It is also called in the beginning
 * when the window is first created. Passes in the width and height of the 
 * window.
 *
 */
void reshape(int width, int height)
{
    height = (height == 0) ? 1 : height;
    width = (width == 0) ? 1 : width;

     /* The 'glViewport' function tells OpenGL to determine how to convert from
     * NDC to screen coordinates given the dimensions of the window. 
     */
    glViewport(0, 0, width, height);

    mouse_scale_x = (float)(p.right_param - p.left_param) / (float)width;
    mouse_scale_y = (float)(p.top_param - p.bottom_param) / (float)height;
    glutPostRedisplay();
}

/* The 'display' function is supposed to handle all the processing of points
 * in world and camera space.
 */
void display(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glLoadIdentity();
    glRotatef(y_view_angle, 1, 0, 0);
    glRotatef(x_view_angle, 0, 1, 0);

    /* Our next step is to specify the inverse rotation of the camera by its
     * orientation angle about its orientation axis:
     */
    glRotatef(-c.cam_orientation_angle * 180.0 / M_PI,
              c.cam_orientation_axis[0], c.cam_orientation_axis[1], 
              c.cam_orientation_axis[2]);
    /* We then specify the inverse translation of the camera by its position using
     * the 'glTranslatef' function
     */
    glTranslatef(-c.cam_position[0], -c.cam_position[1], -c.cam_position[2]);

    /* Transforms the current matrix mode by a rotation matrix  */
    Quat currQuat = getCurrentRotation();
    glMultMatrixd(currQuat.getRotationMatrix().data());

    set_lights();
    draw_objects();
    glutSwapBuffers();
}

/* This function has OpenGL enable its built-in lights to represent our point
 * lights.
 */
void init_lights()
{
    glEnable(GL_LIGHTING);

    num_lights = lights.size();
    for (int i = 0; i < num_lights; ++i)
    {
        int light_id = GL_LIGHT0 + i;
        glEnable(light_id);
        glLightfv(light_id, GL_AMBIENT, lights[i].color);
        glLightfv(light_id, GL_DIFFUSE, lights[i].color);
        glLightfv(light_id, GL_SPECULAR, lights[i].color);
        glLightf(light_id, GL_QUADRATIC_ATTENUATION, lights[i].attenuation_k);
    }
}

/* 
 * The 'set_lights' function positions the lights in camera space. Call this 
 * after the Modelview Matrix has been modified by the necessary camera 
 * transformations
 */
void set_lights()
{
    num_lights = lights.size();

    for (int i = 0; i < num_lights; ++i)
    {
        int light_id = GL_LIGHT0 + i;
        glLightfv(light_id, GL_POSITION, lights[i].position);
    }
}

/* This function has OpenGL render our objects to the display screen. 
 */
void draw_objects()
{
    int num_objects = objects.size();

    for (int i = 0; i < num_objects; ++i)
    {
        glPushMatrix();
        {
            int num_transforms = objects[i].transforms.size();

            /* Modifies the modelview matrix with the desired geometric
             * transformations for this object. 
             */
            for (int j = num_transforms - 1; j >= 0; j--)
            {
                MatTransform trans = objects[i].transforms[j];
                if (trans.mtype.compare("t") == 0)
                {
                    glTranslatef(trans.transform[0], trans.transform[1], 
                            trans.transform[2]);
                }
                else if (trans.mtype.compare("r") == 0)
                {
                    glRotatef(trans.rotation_angle * 180.0 / M_PI, 
                    trans.transform[0], trans.transform[1], trans.transform[2]);
                }
                else if (trans.mtype.compare("s") == 0)
                {
                    glScalef(trans.transform[0], trans.transform[1], 
                        trans.transform[2]);
                }
            }

            glMaterialfv(GL_FRONT, GL_AMBIENT, objects[i].ambient_reflect);
            glMaterialfv(GL_FRONT, GL_DIFFUSE, objects[i].diffuse_reflect);
            glMaterialfv(GL_FRONT, GL_SPECULAR, objects[i].specular_reflect);
            glMaterialf(GL_FRONT, GL_SHININESS, objects[i].shininess);

            glVertexPointer(3, GL_FLOAT, 0, &objects[i].vertex_buffer[0]);
            glNormalPointer(GL_FLOAT, 0, &objects[i].normal_buffer[0]);

            int buffer_size = objects[i].vertex_buffer.size();

            if (!wireframe_mode)
                glDrawArrays(GL_TRIANGLES, 0, buffer_size);
            else
                for (int j = 0; j < buffer_size; j += 3)
                    glDrawArrays(GL_LINE_LOOP, j, 3);
        }
        glPopMatrix();
    }
}

/* This function is meant to respond to mouse clicks and releases. The
 * parameters are:
 * 
 * - int button: the button on the mouse that got clicked or released,
 *               represented by an enum
 * - int state: either 'GLUT_DOWN' or 'GLUT_UP' for specifying whether the
 *              button was pressed down or released up respectively
 * - int x: the x screen coordinate of where the mouse was clicked or released
 * - int y: the y screen coordinate of where the mouse was clicked or released
 */
void mouse_pressed(int button, int state, int x, int y)
{
    /* If the left-mouse button was clicked down, then...
     */
    if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
    {
        /* Store the mouse position in our global variables.
         */
        spx = x;
        spy = y;

        /* Since the mouse is being pressed down, we set our 'is_pressed"
         * boolean indicator to true.
         */
        is_pressed = true;
    }
    /* If the left-mouse button was released up, then...
     */
    else if (button == GLUT_LEFT_BUTTON && state == GLUT_UP)
    {
        last_rotation = current_rotation * last_rotation;
        current_rotation = Quat();
        /* Mouse is no longer being pressed, so set our indicator to false.
         */
        is_pressed = false;
    }
}

/* This function is meant to respond to when the mouse is being moved. There
 * are just two parameters to this function:
 * 
 * - int x: the x screen coordinate of where the mouse was clicked or released
 * - int y: the y screen coordinate of where the mouse was clicked or released
 */
void mouse_moved(int x, int y)
{
    /* If the left-mouse button is being clicked down...
     */
    if (is_pressed)
    {
        /* We update the current mouse position
         */
        cpx = x;
        cpy = y;
        current_rotation = computeRotationQuat();
        glutPostRedisplay();
    }
}

/* Computes the rotation quaternion
 */
Quat getCurrentRotation()
{
    return current_rotation * last_rotation;
}

/* Compute the rotation matrix based on spx, spy, cpx, and cpy
 */
Quat computeRotationQuat()
{
    // First, convert screen coordinates to NDC
    Vector3d scoords = convertScreenToNDC(spx, spy);
    Vector3d ccoords = convertScreenToNDC(cpx, cpy);
    float input = scoords.dot(ccoords) / (scoords.norm() * ccoords.norm());
    float theta = acos(min(input, (float)1.0));
    Vector3d uvec = scoords.cross(ccoords);
    uvec.normalize();
    // Convert rotation angle and rotation axis to quaternion
    Quat quat;
    quat.real = cos(theta / 2);
    if (theta == 0)
    {
        uvec[0] = 1;
        uvec[1] = 1;
        uvec[2] = 1;
    }
    quat.imag[0] = sin(theta / 2) * uvec[0];
    quat.imag[1] = sin(theta / 2) * uvec[1];
    quat.imag[2] = sin(theta / 2) * uvec[2];
    return quat;
}

/* Convert the passed in screen coordinates to NDC coordinates
 */
Vector3d convertScreenToNDC(int px, int py)
{
    float nx = xres / 2;
    float ny = yres / 2;
    float ndc_px = (px / nx) - 1.0;
    float ndc_py = -((py / ny) - 1.0);
    float ndc_pz = 0;
    if (pow(ndc_px, 2) + pow(ndc_py, 2) <= 1)
    {
        ndc_pz = sqrt(1 - pow(ndc_px, 2) - pow(ndc_py, 2));
    }
    Vector3d coords;
    coords << ndc_px, ndc_py, ndc_pz;
    return coords;
}

/* 'deg2rad' function:
 * 
 * Converts given angle in degrees to radians.
 */
float deg2rad(float angle)
{
    return angle * M_PI / 180.0;
}

/* This function is meant to respond to key pressed on the keyboard. The
 * parameters are:
 *
 * - unsigned char key: the character of the key itself or the ASCII value of
 *                      of the key
 * - int x: the x screen coordinate of where the mouse was when the key was pressed
 * - int y: the y screen coordinate of where the mouse was when the key was pressed
 */
void key_pressed(unsigned char key, int x, int y)
{
    /* If 'q' is pressed, quit the program.
     */
    if (key == 'q')
    {
        exit(0);
    }
    /* If 't' is pressed, toggle our 'wireframe_mode' boolean to make OpenGL
     * render our cubes as surfaces of wireframes.
     */
    else if (key == 't')
    {
        wireframe_mode = !wireframe_mode;
        glutPostRedisplay();
    }
    else
    {
        float x_view_rad = deg2rad(x_view_angle);

        /* 'w' for step forward
         */
        if (key == 'w')
        {
            c.cam_position[0] += step_size * sin(x_view_rad);
            c.cam_position[2] -= step_size * cos(x_view_rad);
            glutPostRedisplay();
        }
        /* 'a' for step left
         */
        else if (key == 'a')
        {
            c.cam_position[0] -= step_size * cos(x_view_rad);
            c.cam_position[2] -= step_size * sin(x_view_rad);
            glutPostRedisplay();
        }
        /* 's' for step backward
         */
        else if (key == 's')
        {
            c.cam_position[0] -= step_size * sin(x_view_rad);
            c.cam_position[2] += step_size * cos(x_view_rad);
            glutPostRedisplay();
        }
        /* 'd' for step right
         */
        else if (key == 'd')
        {
            c.cam_position[0] += step_size * cos(x_view_rad);
            c.cam_position[2] += step_size * sin(x_view_rad);
            glutPostRedisplay();
        }
    }
}

/* Reads in vertex and fragment shaders from files.
 */
void readShaders()
{
    string vertProgramSource, fragProgramSource;

    ifstream vertProgFile(vertProgFileName.c_str());

    if (!vertProgFile)
        cerr << "Error opening vertex shader program\n";
    ifstream fragProgFile(fragProgFileName.c_str());
    if (!fragProgFile)
        cerr << "Error opening fragment shader program\n";

    getline(vertProgFile, vertProgramSource, '\0');
    const char *vertShaderSource = vertProgramSource.c_str();
    getline(fragProgFile, fragProgramSource, '\0');
    const char *fragShaderSource = fragProgramSource.c_str();
    char buf[1024];
    GLsizei blah;

    // Initialize shaders
    GLenum vertShader, fragShader;
    shaderProgram = glCreateProgram();
    vertShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertShader, 1, &vertShaderSource, NULL);
    glCompileShader(vertShader);
    GLint isCompiled = 0;
    glGetShaderiv(vertShader, GL_COMPILE_STATUS, &isCompiled);
    if (isCompiled == GL_FALSE)
    {
        GLint maxLength = 0;
        glGetShaderiv(vertShader, GL_INFO_LOG_LENGTH, &maxLength);

        // The maxLength includes the NULL character
        std::vector<GLchar> errorLog(maxLength);
        glGetShaderInfoLog(vertShader, maxLength, &maxLength, &errorLog[0]);

        // Provide the infolog in whatever manor you deem best.
        // Exit with failure.
        for (int i = 0; i < errorLog.size(); i++)
            cout << errorLog[i];
        glDeleteShader(vertShader); // Don't leak the shader.
        return;
    }
    fragShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragShader, 1, &fragShaderSource, NULL);
    glCompileShader(fragShader);

    isCompiled = 0;
    glGetShaderiv(fragShader, GL_COMPILE_STATUS, &isCompiled);
    if (isCompiled == GL_FALSE)
    {
        GLint maxLength = 0;
        glGetShaderiv(fragShader, GL_INFO_LOG_LENGTH, &maxLength);

        // The maxLength includes the NULL character
        std::vector<GLchar> errorLog(maxLength);
        glGetShaderInfoLog(fragShader, maxLength, &maxLength, &errorLog[0]);

        // Provide the infolog in whatever manor you deem best.
        // Exit with failure.
        for (int i = 0; i < errorLog.size(); i++)
            cout << errorLog[i];
        glDeleteShader(fragShader); // Don't leak the shader.
        return;
    }
    glAttachShader(shaderProgram, vertShader);
    glAttachShader(shaderProgram, fragShader);
    glLinkProgram(shaderProgram);
    cerr << "Enabling fragment program: " << gluErrorString(glGetError()) << endl;
    glGetProgramInfoLog(shaderProgram, 1024, &blah, buf);
    cerr << buf;

    cerr << "Enabling program object" << endl;
    glUseProgram(shaderProgram);
    GLint lightsUniformPos = glGetUniformLocation(shaderProgram, "num_lights");
    glUniform1i(lightsUniformPos, num_lights);
}

/* Parses all scene information and object transformations from the file with
 * the passed in name.
 */
void parseContent(string name) {
    // Temporary storage for parsed info
    unordered_map<string, vector<Triple>> objVertices;
    unordered_map<string, vector<Triple>> objNormals;
    unordered_map<string, vector<Face>> objFaces;
    fstream myfile(name);
    readScene(myfile, name, c, p, objects, lights, objVertices, objNormals, 
            objFaces); 
    // Read in transformations to the objects
    readTransforms(myfile, objects, objVertices, objNormals, objFaces);
}
/* The 'main' function:
 *
 * This function is short, but is basically where everything comes together.
 */
int main(int argc, char *argv[])
{
    xres = atoi(argv[2]);
    yres = atoi(argv[3]);

    /* 0 for gouraud shading, 1 for phong */
    int mode = atoi(argv[4]);

    glutInit(&argc, argv);
    /* The following line of code tells OpenGL that we need a double buffer,
     * a RGB pixel buffer, and a depth buffer.
     */
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    /* The following line tells OpenGL to create a program window of size
     * 'xres' by 'yres'.
     */
    glutInitWindowSize(xres, yres);
    /* The following line tells OpenGL to set the program window in the top-left
     * corner of the computer screen (0, 0).
     */
    glutInitWindowPosition(0, 0);
    /* The following line tells OpenGL to name the program window "Test".
     */
    glutCreateWindow("Test");

    parseContent(argv[1]);
    /* Call our 'init' function...
     */
    init();
    if (mode == 0) {
        glShadeModel(GL_SMOOTH);
    }
    else if (mode == 1) {
        /* Read in shaders */
        vertProgFileName = "vertexProgram.glsl";
        fragProgFileName = "fragmentProgram.glsl";
        readShaders();
    }

    /* Specify to OpenGL our display function.
     */
    glutDisplayFunc(display);
    /* Specify to OpenGL our reshape function.
     */
    glutReshapeFunc(reshape);
    /* Specify to OpenGL our function for handling mouse presses.
     */
    glutMouseFunc(mouse_pressed);
    /* Specify to OpenGL our function for handling mouse movement.
     */
    glutMotionFunc(mouse_moved);
    /* Specify to OpenGL our function for handling key presses.
     */
    glutKeyboardFunc(key_pressed);
    /* The following line tells OpenGL to start the "event processing loop". This
     * is an infinite loop where OpenGL will continuously use our display, reshape,
     * mouse, and keyboard functions to essentially run our program.
     */
    glutMainLoop();
}
