#include "parsing.h"
#include "quat.h"
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

void init_lights();
void set_lights();
void draw_objects();

void mouse_pressed(int button, int state, int x, int y);
void mouse_moved(int x, int y);
void key_pressed(unsigned char key, int x, int y);
Quat getCurrentRotation();
Vector3d convertScreenToNDC(int px, int py);

float calcArea(HE* he);
float getSumArea(HEV* vertex);
float getCot(HE* he);
float computeLaplacian(HE* he);
Eigen::SparseMatrix<double> build_F_operator(std::vector<HEV*> *vertices);
void smoothMesh(Object& obj);

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

/* Keeps track of quaternions for Arcball
 */
Quat last_rotation;
Quat current_rotation;

/* Keeps track of the screen resolution */
int xres, yres;
/* The time step used in smoothing */
float h;

///////////////////////////////////////////////////////////////////////////////////////////////////

/* The following are parameters for creating an interactive first-person camera
 * view of the scene.
 */
int spx, spy;
int cpx, cpy;
float mouse_scale_x, mouse_scale_y;

bool is_pressed = false;

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
    glShadeModel(GL_SMOOTH);

    /* Use backface culling
     */
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    
    /* Use depth buffering
     */
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_NORMALIZE);
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    glFrustum(p.left_param, p.right_param,
              p.bottom_param, p.top_param,
              p.near_param, p.far_param);
    glMatrixMode(GL_MODELVIEW);
    
    init_lights();

    last_rotation = Quat();
    current_rotation = Quat();
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
    
    /* Tells OpenGL to determine how to convert from NDC to screen coordinates 
     * given the dimensions of the window. 
     */
    glViewport(0, 0, width, height);
    
    mouse_scale_x = (float) (p.right_param - p.left_param) / (float) width;
    mouse_scale_y = (float) (p.top_param - p.bottom_param) / (float) height;
    glutPostRedisplay();
}

/* Handles all the processing of points in world and camera space.
 */
void display(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glLoadIdentity();

    /* Our next step is to specify the inverse rotation of the camera by its
     * orientation angle about its orientation axis:
     */
    glRotatef(-c.cam_orientation_angle * 180.0 / M_PI,
              c.cam_orientation_axis[0], c.cam_orientation_axis[1], 
              c.cam_orientation_axis[2]);
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
    
    int num_lights = lights.size();
    for(int i = 0; i < num_lights; ++i)
    {
        int light_id = GL_LIGHT0 + i;
        
        glEnable(light_id);
        glLightfv(light_id, GL_AMBIENT, lights[i].color);
        glLightfv(light_id, GL_DIFFUSE, lights[i].color);
        glLightfv(light_id, GL_SPECULAR, lights[i].color);
        glLightf(light_id, GL_QUADRATIC_ATTENUATION, lights[i].attenuation_k);
    }
}

/* Positions the lights in camera space. Call this after the Modelview Matrix 
 * has been modified by the necessary camera transformations
 */
void set_lights()
{
    int num_lights = lights.size();
    
    for(int i = 0; i < num_lights; ++i)
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
    
    for(int i = 0; i < num_objects; ++i)
    {
        glPushMatrix();
        {
            int num_transforms = objects[i].transforms.size();
            
            /* Modifies the modelview matrix with the desired geometric
             * transformations for this object. 
             */
            for(int j = num_transforms - 1; j >= 0; j--)
            {   
                MatTransform trans = objects[i].transforms[j];
                if (trans.mtype.compare("t") == 0) {
                    glTranslatef(trans.transform[0], trans.transform[1], 
                        trans.transform[2]); 
                }
                else if (trans.mtype.compare("r") == 0) {
                    glRotatef(trans.rotation_angle * 180.0 / M_PI, 
                    trans.transform[0], trans.transform[1], trans.transform[2]); 
                }
                else if (trans.mtype.compare("s") == 0) {
                    glScalef(trans.transform[0], trans.transform[1], trans.transform[2]); 
                }   
            }
            glMaterialfv(GL_FRONT, GL_AMBIENT, objects[i].ambient_reflect);
            glMaterialfv(GL_FRONT, GL_DIFFUSE, objects[i].diffuse_reflect);
            glMaterialfv(GL_FRONT, GL_SPECULAR, objects[i].specular_reflect);
            glMaterialf(GL_FRONT, GL_SHININESS, objects[i].shininess);
            
            glVertexPointer(3, GL_FLOAT, 0, &objects[i].vertex_buffer[0]);
            glNormalPointer(GL_FLOAT, 0, &objects[i].normal_buffer[0]);
            
            int buffer_size = objects[i].vertex_buffer.size();
            
            glDrawArrays(GL_TRIANGLES, 0, buffer_size);
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
    if(button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
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
    else if(button == GLUT_LEFT_BUTTON && state == GLUT_UP)
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
    if(is_pressed)
    {   
        /* We update the current px and py
         */
        cpx = x;
        cpy = y;
        current_rotation = Quat::computeRotationQuat(
            convertScreenToNDC(spx, spy), convertScreenToNDC(cpx, cpy));
        glutPostRedisplay();
    }
}

/* Computes the rotation quaternion
 */
Quat getCurrentRotation() {
    return current_rotation * last_rotation;
}

/* Convert the passed in screen coordinates to NDC coordinates
 */
Vector3d convertScreenToNDC(int px, int py) {
    float nx = xres / 2;
    float ny = yres / 2;
    float ndc_px = (px / nx) - 1.0;
    float ndc_py = -((py / ny) - 1.0);
    float ndc_pz = 0;
    if (pow(ndc_px, 2) + pow(ndc_py, 2) <= 1) {
        ndc_pz = sqrt(1 - pow(ndc_px, 2) - pow(ndc_py, 2));
    }
    Vector3d coords;
    coords << ndc_px, ndc_py, ndc_pz;
    return coords;
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
    else if (key == 'm') {
        // Smooth mesh
        for (int i = 0; i < objects.size(); i++) {
            Object obj = objects[i];
            smoothMesh(obj);
            objects[i] = obj;
        }
        glutPostRedisplay();
    }
}

/* Computes the area of the face associated with the given half-edge
 */
float calcArea(HE* he) {
    // Calculate the normal
    HEF *f = he->face;
    HEV *v1 = f->edge->vertex;
    HEV *v2 = f->edge->next->vertex;
    HEV *v3 = f->edge->next->next->vertex;
    Vector3d v1_coords = Vector3d(v1->x, v1->y, v1->z);
    Vector3d v2_coords= Vector3d(v2->x, v2->y, v2->z);
    Vector3d v3_coords= Vector3d(v3->x, v3->y, v3->z);
    Vector3d face_normal = (v2_coords - v1_coords).cross(v3_coords - v1_coords);
    // compute the area of the triangular face
    float face_area = 0.5 * face_normal.norm();
    return face_area;
}

/* Computes the sum of the areas of the faces neighboring a given vertex
 */
float getSumArea(HEV* vertex) {
    HE* he = vertex->out; // get outgoing halfedge from given vertex
    float sumArea = calcArea(he);
    he = he->flip->next;
    while(he != vertex->out) 
    {
        sumArea += calcArea(he);
        he = he->flip->next;
    }
    return sumArea;
}

/* Computes the cotangent of the angle opposite of the given halfedge
*/
float getCot(HE* he) {
    HEF *f = he->face;
    HEV *v1 = f->edge->vertex;
    HEV *v2 = f->edge->next->vertex;
    HEV *v3 = f->edge->next->next->vertex;
    Vector3d v1_coords = Vector3d(v1->x, v1->y, v1->z);
    Vector3d v2_coords= Vector3d(v2->x, v2->y, v2->z);
    Vector3d v3_coords= Vector3d(v3->x, v3->y, v3->z);
    Vector3d vec1 = v1_coords - v3_coords;
    Vector3d vec2 = v2_coords - v3_coords;
    // Use the dot product over magnitude of cross product formula
    return abs(vec1.dot(vec2)) / (vec1.cross(vec2)).norm();
}

/* Computes the 0.5 * (cot alpha_j + cot beta_j) part of the discrete 
 * Laplacian for the given halfedge. 
 */ 
float computeLaplacian(HE* he) {
    float cot_alpha = getCot(he);
    float cot_beta = getCot(he->flip);
    return cot_alpha + cot_beta;
}

/* Compute the F operator. First build the discrete Laplacian by multiplying
 * a matrix D whose diagonal is filled with (h / 2A) by a second matrix X to 
 * form a product P. For X, the value at (i, j) for i not equal to j is
 * (cot alpha_j + cot beta_j) if i and j form an edge. If i = j, the value is 
 * the sum over all (cot alpha_j + cot beta_j). Thus each element of P is 
 * h * (delta x)_i = (h / 2A) (sum of (cot alpha + cot beta) (x_j - x_i)).
 * We then return the identity matrix minus P.
 */
Eigen::SparseMatrix<double> build_F_operator(vector<HEV*> *vertices )
{
    // Due to 1-indexing of obj files, index 0 of our list doesn’t actually 
    // contain a vertex
    int num_vertices = vertices->size() - 1;
    // initialize a sparse matrix to represent our F operator
    Eigen::SparseMatrix<double> F( num_vertices, num_vertices );
    Eigen::SparseMatrix<double> mult( num_vertices, num_vertices );
    Eigen::SparseMatrix<double> sparseI( num_vertices, num_vertices );

    // reserve room for 7 non-zeros per row of B
    F.reserve( Eigen::VectorXi::Constant( num_vertices, 7 ) );

    for( int i = 1; i < vertices->size(); ++i )
    {   
        HE *he = vertices->at(i)->out;
        float A = getSumArea( vertices->at(i));
        bool first = true;
        float sum = 0;
        while( he != vertices->at(i)->out|| first) {
            // get index of adjacent vertex to v_i
            int j = he->next->vertex->index; 
            float cot = computeLaplacian(he);
            F.insert( i-1, j-1 ) = cot;
            sum += cot;
            he = he->flip->next;
            first = false;
        }
        F.insert( i-1, i-1 ) = -sum;
        // Prevent dividing by A if A is near 0
        if (abs(A) > 1e-8) {
            mult.insert(i-1, i-1) = h / (2 * A);
        }
        else {
            mult.insert(i-1, i-1) = 0;
        }
        sparseI.insert(i-1, i-1) = 1.0;
    }
    Eigen::SparseMatrix<double> newF = sparseI - mult * F;
    newF.makeCompressed();
    return newF;
}

/* Constructs the matrix operator F and uses Eigen's sparse solver to solve for
 * the new x, y, z coordinates. Recomputes normals based on the new coordinates
 * and sets the vertex and normal buffers appropriately
 */ 
void smoothMesh(Object& obj)
{
    vector<HEV*> *vertices = &obj.he_vertices;
    // get our matrix representation of F
    Eigen::SparseMatrix<double> F = build_F_operator( vertices );

    int num_vertices = vertices->size() - 1;

    Eigen::SparseLU<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> 
    solver;

    solver.analyzePattern(F);
    solver.factorize(F);

    // initialize our vector representation of rho, which is the original 
    // coordinate
    Eigen::VectorXd x_vector(num_vertices);
    Eigen::VectorXd y_vector(num_vertices);
    Eigen::VectorXd z_vector(num_vertices);
    for( int i = 1; i < vertices->size(); ++i ) {
        x_vector[i-1] = vertices->at(i)->x; 
        y_vector[i-1] = vertices->at(i)->y; 
        z_vector[i-1] = vertices->at(i)->z; 
    }

    // Have Eigen solve for our xh, yh, zh vectors
    Eigen::VectorXd xh_vector(num_vertices);
    Eigen::VectorXd yh_vector(num_vertices);
    Eigen::VectorXd zh_vector(num_vertices);
    xh_vector = solver.solve(x_vector);
    yh_vector = solver.solve(y_vector);
    zh_vector = solver.solve(z_vector);
    
    vector<Vec3f> vertex_buffer;
    vector<Vec3f> normal_buffer;
    // Set the new vertex positions
    for (int i = 1; i < vertices->size(); ++i) {
        vertices->at(i)->x = xh_vector[i - 1];
        vertices->at(i)->y = yh_vector[i - 1];
        vertices->at(i)->z = zh_vector[i - 1];
    }
    // Recompute the normals 
    for (int i = 1; i < vertices->size(); ++i) {
        vertices->at(i)->normal = calc_vertex_normal(vertices->at(i));
    }
    for (HEF* f : obj.he_faces) {
        HEV *v1 = f->edge->vertex;
        HEV *v2 = f->edge->next->vertex;
        HEV *v3 = f->edge->next->next->vertex;
        Vec3f v1_coords{(float) v1->x, (float) v1->y, (float) v1->z};
        Vec3f v2_coords{(float) v2->x, (float) v2->y, (float) v2->z};
        Vec3f v3_coords{(float) v3->x, (float) v3->y, (float) v3->z};
        vertex_buffer.push_back(v1_coords);
        vertex_buffer.push_back(v2_coords);
        vertex_buffer.push_back(v3_coords);
        normal_buffer.push_back(v1->normal);
        normal_buffer.push_back(v2->normal);
        normal_buffer.push_back(v3->normal);
    }
    obj.vertex_buffer = vertex_buffer;
    obj.normal_buffer = normal_buffer;
}

/* Parses a scene file and stores the contents into Camera, Perspective, and
 * Object objects 
 */ 
void parseContent(string name) {
    /* Temporary storage for parsed info */
    unordered_map<string, Mesh_Data> objMesh;
    unordered_map<string, vector<HEV*>> objVertices;
    unordered_map<string, vector<HEF*>> objFaces;
    fstream myfile(name);
    if (! myfile.good()) {
        cout << "Scene file does not exist. Exiting" << endl;
        exit(1);
    }
    readScene(myfile, c, p, objects, lights, objMesh, objVertices, objFaces); 
    // Read in transformations to the objects
    readTransforms(myfile, objects, objVertices, objFaces);
}

/* The 'main' function:
 *
 * This function is short, but is basically where everything comes together.
 */
int main(int argc, char* argv[])
{
    if (argc != 5 || ! isdigit(*argv[2]) || ! isdigit(*argv[3]) || 
        ! isdigit(*argv[4])) {
        cout << "Wrong command line arguments. The correct format is: " << endl;
        cout << "./smooth [scene file] [xres] [yres] [h]"<< endl;
        cout << "scene file: string" << endl;
        cout << "xres: int" << endl;
        cout << "yres: int" << endl;
        cout << "h: float" << endl;
        return -1;
    }
    xres = atoi(argv[2]);
    yres = atoi(argv[3]);
    h = atof(argv[4]);
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(xres, yres);
    glutInitWindowPosition(0, 0);
    glutCreateWindow("Test");
    
    parseContent(argv[1]);
    init();
    
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutMouseFunc(mouse_pressed);
    glutMotionFunc(mouse_moved);
    glutKeyboardFunc(key_pressed);
    glutMainLoop();
}
