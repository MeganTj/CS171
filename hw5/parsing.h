#include <iostream>
#include <fstream>
#include <sstream> 
#include <vector> 
#include <unordered_map>
#include <cmath>
#include <float.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "halfedge.h"

using namespace Eigen;
using namespace std;

/* The following struct is used for representing a point light.
 *
 * Note that the position is represented in homogeneous coordinates rather than
 * the simple Cartesian coordinates that we would normally use. This is because
 * OpenGL requires us to specify a w-coordinate when we specify the positions
 * of our point lights. We specify the positions in the 'set_lights' function.
 */
struct Point_Light
{
    /* Index 0 has the x-coordinate
     * Index 1 has the y-coordinate
     * Index 2 has the z-coordinate
     * Index 3 has the w-coordinate
     */
    float position[4];
    
    /* Index 0 has the r-component
     * Index 1 has the g-component
     * Index 2 has the b-component
     */
    float color[3];
    
    /* This is our 'k' factor for attenuation as discussed in the lecture notes
     * and extra credit of Assignment 2.
     */
    float attenuation_k;
};

/* The following struct is used for representing points and normals in world
 * coordinates.
 *
 * Notice how we are using this struct to represent points, but the struct
 * lacks a w-coordinate. Fortunately, OpenGL will handle all the complications
 * with the homogeneous component for us when we have it process the points.
 * We do not actually need to keep track of the w-coordinates of our points
 * when working in OpenGL.
 */
struct Triple
{
    float x;
    float y;
    float z;
    // Triple(float x, float y, float z) : x(x), y(y), z(z){};
};

struct MatTransform {
    float transform[3];
    /* Angle in degrees.
     */
    float rotation_angle;
    string mtype;
};


/* Stores an object with a label, the copy number (starting from 1), and vectors
 * storing vertices and faces as well as transformation matrices to be applied
 * to the object.
 */
struct Object{
    vector<Vec3f> vertex_buffer;
    vector<Vec3f> normal_buffer;
    vector<HEV*> he_vertices;
    vector<HEF*> he_faces;
    vector<MatTransform> transforms;
    /* Index 0 has the r-component
     * Index 1 has the g-component
     * Index 2 has the b-component
     */
    float ambient_reflect[3];
    float diffuse_reflect[3];
    float specular_reflect[3];
    
    float shininess;
};

struct Camera {
    /* Index 0 has the x-coordinate
    * Index 1 has the y-coordinate
    * Index 2 has the z-coordinate
    */
    float cam_position[3];
    float cam_orientation_axis[3];
    float cam_orientation_angle;
};

struct Perspective {
    float near_param;
    float far_param;
    float left_param;
    float right_param;
    float top_param;
    float bottom_param;
    Eigen::Matrix4d getProjection();
};

Vec3f calc_vertex_normal(HEV *vertex);
Vec3f calc_weighted_normal(HE* he);
void readFile(string name, vector<Vertex>& vertices, vector<Face>& faces);
void readObj(fstream& myfile, unordered_map<string, Mesh_Data>& objMesh, 
unordered_map<string, vector<HEV*>>& objVertices, 
unordered_map<string, vector<HEF*>>& objFaces);
void readScene(fstream& myfile, Camera& c, Perspective& p, vector<Object>& objects,
vector<Point_Light>& lights, unordered_map<string, Mesh_Data>& objMesh, 
unordered_map<string, vector<HEV*>>& objVertices, unordered_map<string, vector<HEF*>>& objFaces);
void readTransforms(fstream& myfile, vector<Object>& objects, 
                    unordered_map<string, vector<HEV*>>& objVertices, 
                    unordered_map<string, vector<HEF*>>& objFaces);
pair<int, int> convertNDCToScreen(Vector3d coords, int xres, int yres);