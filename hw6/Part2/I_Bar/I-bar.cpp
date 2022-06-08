#include "helper.h"
/* The following 2 headers contain all the main functions, data structures, and
 * variables that allow for OpenGL development.
 */
#include <GL/glew.h>
#include <GL/glut.h>

/* Used in converting between degrees and radians.
 * Besides the use of 'M_PI', the trigometric functions also show up a lot in
 * graphics computations.
 */
#include <math.h>
#define _USE_MATH_DEFINES

/* iostream and vector are standard libraries that are just generally useful.
 */
#include <iostream>
#include <vector>

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
void transformFrame();

void drawIBar();
void key_pressed(unsigned char key, int x, int y);

///////////////////////////////////////////////////////////////////////////////////////////////////

Frames frames;
int frameIndex;

/* Keeps track of the screen resolution */
int xres, yres;

///////////////////////////////////////////////////////////////////////////////////////////////////

GLUquadricObj *quadratic;
///////////////////////////////////////////////////////////////////////////////////////////////////

/* 'init' function:
 * 
 * As you would expect, the 'init' function initializes and sets up the
 * program. It should always be called before anything else.
 *
 */
void init(void)
{
    glShadeModel(GL_SMOOTH);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_NORMALIZE);
    
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    glFrustum(-1.0, 1.0,
              -1.0, 1.0,
              1.0, 60.0);
    glMatrixMode(GL_MODELVIEW);
    quadratic = gluNewQuadric();
}

/* Tells the program how to react whenever the program window is resized. 
 * It is also called in the beginning when the window is first created. 
 * Passes in the width and height of the window.
 *
 */
void reshape(int width, int height)
{
    height = (height == 0) ? 1 : height;
    width = (width == 0) ? 1 : width;

    glViewport(0, 0, width, height);
    glutPostRedisplay();
}

/* Transforms by the frame transformation that frameIndex currently points to 
 */
void transformFrame() {
    FrameTransform curr = frames.transforms[frameIndex];
    glTranslatef(curr.translation[0], curr.translation[1], curr.translation[2]);
    glMultMatrixd(curr.rotation.getRotationMatrix().data());
    glScalef(curr.scale[0], curr.scale[1], curr.scale[2]);
}

/* Displays the IBar
 */
void drawIBar()
{
    /* Parameters for drawing the cylinders */
    float cyRad = 0.2, cyHeight = 1.0;
    int quadStacks = 4, quadSlices = 4;
    
    glPushMatrix();
    glColor3f(0, 0, 1);
    glTranslatef(0, cyHeight, 0);
    glRotatef(90, 1, 0, 0);
    gluCylinder(quadratic, cyRad, cyRad, 2.0 * cyHeight, quadSlices, quadStacks);
    glPopMatrix();
    
    glPushMatrix();
    glColor3f(0, 1, 1);
    glTranslatef(0, cyHeight, 0);
    glRotatef(90, 0, 1, 0);
    gluCylinder(quadratic, cyRad, cyRad, cyHeight, quadSlices, quadStacks);
    glPopMatrix();
    
    glPushMatrix();
    glColor3f(1, 0, 1);
    glTranslatef(0, cyHeight, 0);
    glRotatef(-90, 0, 1, 0);
    gluCylinder(quadratic, cyRad, cyRad, cyHeight, quadSlices, quadStacks);
    glPopMatrix();
    
    glPushMatrix();
    glColor3f(1, 1, 0);
    glTranslatef(0, -cyHeight, 0);
    glRotatef(-90, 0, 1, 0);
    gluCylinder(quadratic, cyRad, cyRad, cyHeight, quadSlices, quadStacks);
    glPopMatrix();
    
    glPushMatrix();
    glColor3f(0, 1, 0);
    glTranslatef(0, -cyHeight, 0);
    glRotatef(90, 0, 1, 0);
    gluCylinder(quadratic, cyRad, cyRad, cyHeight, quadSlices, quadStacks);
    glPopMatrix();
}

/* The 'display' function is supposed to handle all the processing of points
 * in world and camera space.
 */
void display(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glLoadIdentity();

    /* We then specify the inverse translation of the camera by its position using
     * the 'glTranslatef' function
     */
    glTranslatef(0, 0, -40);

    /* Use the appropriate transformation for the current frame */
    transformFrame();
    drawIBar();
    glutSwapBuffers();
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
    if(key == 'q')
    {
        exit(0);
    }
    else if(key == 'n')
    {   
        frameIndex++;
        if (frameIndex == frames.transforms.size()) {
            frameIndex = 0;
        }
        glutPostRedisplay();
    }
}

/* Parses a file with the keyframe transformations, then interpolates to find
 * the transformations for the rest of the frames.
 */
void parseContent(string name) {
    fstream myfile(name);
    if (! myfile.good()) {
        cout << "Script file does not exist. Exiting" << endl;
        exit(1);
    }
    frames = readFrameTransforms(myfile);
    frames.interpolate();
    frameIndex = 0;
}

/* The 'main' function:
 *
 * This function is short, but is basically where everything comes together.
 */
int main(int argc, char* argv[])
{
    if (argc != 4 || ! isdigit(*argv[2]) || ! isdigit(*argv[3])) {
        cout << "Wrong command line arguments. The correct format is: " << endl;
        cout << "./keyframe [script file] [xres] [yres]"<< endl;
        cout << "script file: string" << endl;
        cout << "xres: int" << endl;
        cout << "yres: int" << endl;
        return -1;
    }
    xres = atoi(argv[2]);
    yres = atoi(argv[3]);

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(xres, yres);
    glutInitWindowPosition(0, 0);

    glutCreateWindow("Test");
    
    parseContent(argv[1]);
    init();
    
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(key_pressed);
    glutMainLoop();
}
