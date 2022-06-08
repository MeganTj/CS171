#include "part1.h"
#include <unordered_map>
#include <cmath>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

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
    /* Applies the transformations specified in the matrices attribute and 
     * prints out the transformed vertices
     */
    void applyAndPrint();
};