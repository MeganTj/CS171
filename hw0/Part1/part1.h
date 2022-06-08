#include <iostream>
#include <fstream>
#include <sstream> 
#include <vector> 

using namespace std;

/* Contains three floats that are a vertex's x, y, and z coordinates.
 */ 
struct Vertex {
    float x;
    float y;
    float z;
    Vertex() : x(0), y(0), z(0) {};
    Vertex(float x,  float y, float z) : x(x), y(y), z(z) {};
};

/* Contains the indices of vertices (which are 1-indexed) that comprise a face
 */
struct Face {
    int v1;
    int v2;
    int v3;
    Face(int v1, int v2, int v3) : v1(v1), v2(v2), v3(v3) {};
};

/* Reads in vertices and faces from a .obj file */
void readFile(string name, vector<Vertex>& vertices,  vector<Face>& faces);
/* Prints out vertices and faces stored in a .obj file. */
void printFile(string name, vector<Vertex>& vertices,  vector<Face>& faces);