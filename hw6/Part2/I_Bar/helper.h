#include <iostream>
#include <fstream>
#include <sstream> 
#include <vector> 
#include <unordered_map>
#include <cmath>
#include <float.h>
#include <Eigen/Dense>
#include "quat.h"

using namespace Eigen;
using namespace std;

/* Contains the translation, scaling, and rotation of each point in the 
 * scene for a given frame
 */
struct FrameTransform {
    float translation[3];
    float scale[3];
    /* The angle is the last numerical value */
    Quat rotation;
};

/* Keeps track of the interval length between keyframes and the FrameTransform
 * associated with each frame
 */
struct Frames {
    vector<int> gaps;
    vector<FrameTransform> transforms;
    /* Use Catmull-Rom splines to interpolate between keyframes */
    void interpolate();
    /* Computes a single interpolated frame */
    FrameTransform interpolateFT(double u, Matrix4d B, FrameTransform ft1, 
            FrameTransform ft2, FrameTransform ft3, FrameTransform ft4);
    /* Computes a single interpolated value */
    double interpolateVal(double u, Matrix4d B, Vector4d p);
};

pair<int, int> getPair(string str);
Frames readFrameTransforms(fstream& myfile);