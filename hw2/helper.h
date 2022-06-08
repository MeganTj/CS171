#include <iostream>
#include <fstream>
#include <sstream> 
#include <vector> 
#include <unordered_map>
#include <cmath>
#include <float.h>
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
    Vector3d getPos();
};

/* Stores normal values
 */
struct Normal {
    float x;
    float y;
    float z;
    Normal() : x(0), y(0), z(0) {};
    Normal(float x,  float y, float z) : x(x), y(y), z(z) {};
    void normalize();
    /* Transforms this normal using the passed in transformation matrix */
    Vector3d transformNormal(Matrix4d tmatrix);
};

/* Contains pairs consisting of the index of the vertex as well as its the
 * index of its normal 
 */
struct Face {
    pair<int, int> v1;
    pair<int, int>  v2;
    pair<int, int>  v3;
    Face(pair<int, int>  v1, pair<int, int>  v2, pair<int, int>  v3) : 
        v1(v1), v2(v2), v3(v3) {};
};

/* Contains r, g, b values
 */
struct Color {
    float r;
    float g;
    float b;
    Color() : r(0), g(0), b(0) {};
    Color(float r, float g, float b) : r(r), g(g), b(b) {};
    Color(Vector3d vec) : r(vec[0]), g(vec[1]), b(vec[2]){};
    Vector3d getVector();
};

/* Contains the position of the light, its color, and attenuation value
 */
struct Light {
    float x;
    float y;
    float z;
    Color c;
    float atten;
    Vector3d getPosVector();
};

/* An image object containing a grid of color values and a buffer grid, both of
 * which are yres by xres.
 */
struct Image {
    int xres;
    int yres;
    vector<vector<Color>> grid;
    vector<vector<double>> buffer;
    Image(int xres, int yres);
    void fillGrid(int x, int y, Color c);
    void fillBuffer(int x, int y, double val);
    void outputGrid();
};

/* An object representing a matrix transformation.
 */
struct MatTransform {
    Matrix4d mat;
    string mtype;
};

/* Stores an object with a label, the copy number (starting from 1), and vectors
 * storing vertices and faces as well as transformation matrices to be applied
 * to the object.
 */
struct Object{
    string label;
    int copyNum;
    vector<Vertex> vertices;
    vector<Normal> normals;
    vector<Face> faces;
    vector<MatTransform> matrices;
    Color ambient;
    Color diffuse;
    Color specular;
    float phong;
    Object(string label, int copyNum, vector<Vertex> v, vector<Normal> vn, 
    vector<Face> f) :  label(label), copyNum(copyNum), vertices(v), 
                        normals(vn), faces(f) {};
    Matrix4d getTransformMatrix(bool translate);
    void applyAndTransform();
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
    Vector3d getPosVector();
};

/* Stores values for creating the projection matrix
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


pair<int, int> getPair(string str);
void readFile(string name, vector<Vertex>& vertices, vector<Normal>& normals, 
            vector<Face>& faces);
void readObj(fstream& myfile, string path, unordered_map<string, vector<Vertex>>& 
        objVertices, unordered_map<string, vector<Normal>>& objNormals,
        unordered_map<string, vector<Face>>& objFaces);
void readTransforms(fstream& myfile, vector<Object>& objects, 
                    unordered_map<string, vector<Vertex>> objVertices, 
                    unordered_map<string, vector<Normal>> objNormals,
                    unordered_map<string, vector<Face>> objFaces);
void readScene(fstream& myfile, string path, Camera& c, Perspective& p, 
                vector<Object>& objects,vector<Light>& lights, 
                unordered_map<string, vector<Vertex>>& objVertices, 
                unordered_map<string, vector<Normal>>& objNormals, 
                unordered_map<string, vector<Face>>& objFaces);
Vector3d convertWorldToNDC(Vertex v, Object obj, Camera c, Perspective p);
pair<int, int> convertNDCToScreen(Vector3d coords, int xres, int yres);
Matrix4d getRotationMatrix(float x, float y, float z, float theta);