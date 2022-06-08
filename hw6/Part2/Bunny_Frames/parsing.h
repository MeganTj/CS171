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
struct Object{
    vector<Vertex> vertices;
    vector<Face> faces;
    void outputObj(string name);
};

pair<int, int> getPair(string str);
double interpolateVal(double u, Matrix4d B, Vector4d p);
Object readObj(string name);
Object interpolateObj(double u, Matrix4d B, Object obj1, Object obj2, 
                    Object obj3, Object obj4);
void interpolateObjects(vector<Object>& objects);