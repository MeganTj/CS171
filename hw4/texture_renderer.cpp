#include "parsing.h"
#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#define GL_GLEXT_PROTOTYPES 1

#include <math.h>
#define _USE_MATH_DEFINES

using namespace std;

extern GLenum readpng(const char *filename);
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

void mouse_pressed(int button, int state, int x, int y);
void mouse_moved(int x, int y);
void key_pressed(unsigned char key, int x, int y);
Quat getCurrentRotation();
Quat computeRotationQuat();
Vector3d convertScreenToNDC(int px, int py);

///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////

int num_lights;
static const float gridSize = 100;

/* Keeps track of quaternions for Arcball
 */
Quat last_rotation;
Quat current_rotation;

/* Keeps track of the screen resolution */
int xres, yres;

/* Constants related to shading
 */
GLenum shaderProgram;
GLenum texMap, normalMap;
GLuint tanBuffer;
GLint texUniformPos, normalUniformPos;
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
    GLfloat pos[] = {7.0, 2.0, 3.0, 1.0};
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 1);
    glEnable(GL_COLOR_MATERIAL);
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    glColorMaterial(GL_FRONT, GL_DIFFUSE);
    glLightfv(GL_LIGHT0, GL_POSITION, pos);
    glEnable(GL_DEPTH_TEST);
    glewInit();
}

/* The 'reshape' function is supposed to tell your program how to react
 * whenever the program window is resized. It is also called in the beginning
 * when the window is first created. Passes in the width and height of the 
 * window.
 */
void reshape(int width, int height)
{
    height = (height == 0) ? 1 : height;
    width = (width == 0) ? 1 : width;
    glViewport(0, 0, width, height);
    glutPostRedisplay();
}

/*  The 'display' function is supposed to handle all the processing of points
 * in world and camera space.
 */
void display(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    glRotatef(y_view_angle, 1, 0, 0);
    glRotatef(x_view_angle, 0, 1, 0);

    /* Transforms the current matrix mode by a rotation matrix  */
    Quat currQuat = getCurrentRotation();
    glMultMatrixd(currQuat.getRotationMatrix().data());

    glPushMatrix();

    /* Set the vertex and texture coordinate positions for the shaders to use.
     */
    float vertexArr[] = {-0.75f, 0.75f, 0.0f,
                         0.75f, 0.75f, 0.0f,
                         0.75f, -0.75f, 0.0f,
                         0.75f, -0.75f, 0.0f,
                         -0.75f, -0.75f, 0.0f,
                        -0.75f, 0.75f, 0.0f};
    glVertexPointer(3, GL_FLOAT, 0, &vertexArr);
    int buffer_size = sizeof(vertexArr) / sizeof(float);

    float texCoordArr[] = {0.0f, 1.0f,
                           1.0f, 1.0f,
                           1.0f, 0.0f, 
                           1.0f, 0.0f, 
                           0.0f, 0.0f,
                           0.0f, 1.0f};
    glTexCoordPointer(2, GL_FLOAT, 0, &texCoordArr);

    glGenBuffers(1, &tanBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, tanBuffer);
    float tangentArr[] = {1.0f, 0.0f, 0.0f,
                          1.0f, 0.0f, 0.0f,
                          1.0f, 0.0f, 0.0f,
                          1.0f, 0.0f, 0.0f,
                          1.0f, 0.0f, 0.0f,
                          1.0f, 0.0f, 0.0f};
    GLint tangentLoc = glGetAttribLocation(shaderProgram, "tangent");
    glBufferData(GL_ARRAY_BUFFER, sizeof(tangentArr), &tangentArr, GL_STATIC_DRAW);
    glVertexAttribPointer(tangentLoc, 3, GL_FLOAT, 0, 0, 0);
    glEnableVertexAttribArray(tangentLoc);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    // Weird hack to prevent visual glitches when rotating
    float normalArr[] = {0.0f, 0.0f, 1.0f,
                         0.0f, 0.0f, 1.0f,
                         0.0f, 0.0f, 1.0f,
                         0.0f, 0.0f, 1.0f,
                         0.0f, 0.0f, 1.0f,
                        0.0f, 0.0f, 1.0f};
    glDrawArrays(GL_TRIANGLES, 0, buffer_size);
    glPopMatrix();
    glutSwapBuffers();
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
        /* We update the current px and py
         */
        cpx = x;
        cpy = y;
        current_rotation = computeRotationQuat();
        /* Tell OpenGL that it needs to re-render our scene with the new camera
         * angles.
         */
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

/* 'key_pressed' function:
 * 
 * This function is meant to respond to key pressed on the keyboard. The
 * parameters are:
 *
 * - unsigned char key: the character of the key itself or the ASCII value of
 *                      of the key
 * - int x: the x screen coordinate of where the mouse was when the key was pressed
 * - int y: the y screen coordinate of where the mouse was when the key was pressed
 *
 * Our function is pretty straightforward as you can see below. We also do not make
 * use of the 'x' and 'y' parameters.
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
        /* Tell OpenGL that it needs to re-render our scene with the cubes
         * now as wireframes (or surfaces if they were wireframes before).
         */
        glutPostRedisplay();
    }
}

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

    texUniformPos = glGetUniformLocation(shaderProgram, "texture");
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texMap);
    glUniform1i(texUniformPos, 0);

    normalUniformPos = glGetUniformLocation(shaderProgram, "heightmap");
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, normalMap);
    glUniform1i(normalUniformPos, 1);
}

/* The 'main' function:
 *
 * This function is short, but is basically where everything comes together.
 */
int main(int argc, char *argv[])
{

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    /* The following line tells OpenGL to create a program window of size
     * 'xres' by 'yres'.
     */
    xres = 800;
    yres = 800;
    glutInitWindowSize(xres, yres);
    /* The following line tells OpenGL to set the program window in the top-left
     * corner of the computer screen (0, 0).
     */
    glutInitWindowPosition(0, 0);
    /* The following line tells OpenGL to name the program window "Test".
     */
    glutCreateWindow("Test");

    /* Call our 'init' function...
     */
    init();
    /* Read in the color texture file and the normal map file */
    if (!(texMap = readpng(argv[1])))
        exit(1);
    if (!(normalMap = readpng(argv[2])))
        exit(1);
    /* Read in shaders */
    vertProgFileName = "texture_vertexProgram.glsl";
    fragProgFileName = "texture_fragmentProgram.glsl";
    readShaders();

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
