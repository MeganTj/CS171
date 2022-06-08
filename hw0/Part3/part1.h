#include <iostream>
#include <fstream>
#include <sstream> 
#include <vector> 

using namespace std;

struct Vertex {
    float x;
    float y;
    float z;
    Vertex() : x(0), y(0), z(0) {};
    Vertex(float x,  float y, float z) : x(x), y(y), z(z) {};
};

struct Face {
    int v1;
    int v2;
    int v3;
    Face(int v1, int v2, int v3) : v1(v1), v2(v2), v3(v3) {};
};

void readFile(string name, vector<Vertex>& vertices,  vector<Face>& faces);