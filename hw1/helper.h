#include <iostream>
#include <fstream>
#include <sstream> 
#include <vector> 
#include <unordered_map>
#include <cmath>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

/* Stores vertex coordinates
 */ 
struct Vertex {
    float x;
    float y;
    float z;
    Vertex() : x(0), y(0), z(0) {};
    Vertex(float x,  float y, float z) : x(x), y(y), z(z) {};
};

/* Stores vertex indices (which are 1-indexed)
 */
struct Face {
    int v1;
    int v2;
    int v3;
    Face(int v1, int v2, int v3) : v1(v1), v2(v2), v3(v3) {};
};

/* Stores an object with a label, the copy number (starting from 1), and vectors
 * storing vertices and faces as well as transformation matrices to be applied
 * to the object.
 */
struct Object{
    string label;
    int copyNum;
    vector<Vertex> vertices;
    vector<Face> faces;
    vector<Matrix4d> matrices;
    Object(string label, int copyNum, vector<Vertex> v, vector<Face> f) :  
    label(label), copyNum(copyNum), vertices(v), faces(f) {};
};

/* Stores camera characteristics
 */
struct Camera {
    float posx;
    float posy;
    float posz;
    float orx;
    float ory;
    float orz;
    float theta;
    Eigen::Matrix4d getWorldToCamera();
};

/* Stores values for creating the perspective matrix
 */
struct Perspective {
    float n;
    float f;
    float l;
    float r;
    float t;
    float b;
    Eigen::Matrix4d getProjection();
};


void readFile(string name, vector<Vertex>& vertices,  vector<Face>& faces);

void readObj(fstream& myfile, unordered_map<string, vector<Vertex>>& objVertices,
             unordered_map<string, vector<Face>>& objFaces);
void readTransforms(fstream& myfile, vector<Object>& objects, 
                    unordered_map<string, vector<Vertex>> objVertices, 
                    unordered_map<string, vector<Face>> objFaces);
void readScene(fstream& myfile, Camera& c, Perspective& p, 
                vector<Object>& objects, 
                unordered_map<string, vector<Vertex>>& objVertices, 
                unordered_map<string, vector<Face>>& objFaces);
Matrix4d getRotationMatrix(float x, float y, float z, float theta);