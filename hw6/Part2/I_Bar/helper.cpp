#include "helper.h"

/* Takes in a script file specifying keyframes and returns an object containing
 * each keyframe's transformations
 */ 
Frames readFrameTransforms(fstream& myfile) {
    string line;
    getline(myfile, line);
    istringstream iss(line);
    int length;
    iss >> length;
    int prevIndex = 0;
    Frames frames;
    vector<int> gaps;
    vector<FrameTransform> transforms(length);
    while (! myfile.eof()) {
        getline(myfile, line);
        iss.str(line);
        iss.clear();
        string token; 
        iss >> token;
        int index;
        iss >> index;

        /* Read in translation, scaling, and rotation */
        FrameTransform ft;
        getline(myfile, line);
        iss.str(line);
        iss.clear();
        iss >> token;
        iss >> ft.translation[0] >> ft.translation[1] >> ft.translation[2];

        getline(myfile, line);
        iss.str(line);
        iss.clear();
        iss >> token;
        iss >> ft.scale[0] >> ft.scale[1] >> ft.scale[2];

        getline(myfile, line);
        iss.str(line);
        iss.clear();
        iss >> token;
        float rotation[4];
        iss >> rotation[0] >> rotation[1] >> rotation[2] >> 
        rotation[3];

        /* Make sure that the rotation axis is a unit vector */
        float norm = sqrt(pow(rotation[0], 2) + pow(rotation[1], 2) + 
        pow(rotation[2], 2));
        rotation[0] /= norm;
        rotation[1] /= norm;
        rotation[2] /= norm;
        ft.rotation = Quat(rotation);

        transforms[index] = ft;
        if (index != 0) {
            gaps.push_back(index - prevIndex);
            prevIndex = index;
        }
    }
    gaps.push_back(length - prevIndex);
    frames.gaps = gaps;
    frames.transforms = transforms;
    myfile.close();
    return frames;
}

/* Produce a smooth interpolation across keyframes using Catmull-Rom splines.
 * Modifies the transform vector to contain FrameTransforms for non-keyframe
 * frames. 
 */
void Frames::interpolate() {
    int totalIndex = 0;
    // Create the B matrix 
    Matrix4d B;
    B << 0, 2, 0, 0,
         -1, 0, 1, 0,
         2, -5, 4, -1,
         -1, 3, -3, 1;
    B *= 0.5;
    // ft1 to ft4 correspond with K_{i-1}, K_i, K_{i+1}, K_{i+2}
    for (int i = 0; i < gaps.size(); i++) {
        FrameTransform ft2 = transforms[totalIndex];
        FrameTransform begin = transforms[0];
        FrameTransform last = transforms[transforms.size() - 
                                gaps[gaps.size() - 1]];
        for (int j = 1; j < gaps[i]; j++) {
            double u = (double) j / (double) (gaps[i]);
            FrameTransform ft1;
            FrameTransform ft4;
            FrameTransform ft3;
            if (i == 0) {
                // If the frame is in the first interval (between K_0 and K_1),
                // set K_{i-1} to the last frame
                ft1 = last;
                ft3 = transforms[totalIndex + gaps[i]];
                ft4 = transforms[totalIndex + gaps[i] + gaps[i + 1]];
            }
            else if (i == gaps.size() - 2) {
                // If the frame is in the second to last interval, set K_{i+2}
                // to K_0
                ft1 = transforms[totalIndex - gaps[i - 1]];
                ft3 = transforms[totalIndex + gaps[i]];
                ft4 = begin;
            }
            else if (i == gaps.size() - 1) {
                // If the frame is in the last interval, set K_{i+1} to K_0, 
                // K_{i+2} to K_1
                ft1 = transforms[totalIndex - gaps[i - 1]];
                ft3 = begin;
                ft4 = transforms[gaps[0]];
            }
            else {
                ft1 = transforms[totalIndex - gaps[i - 1]];
                ft3 = transforms[totalIndex + gaps[i]];
                ft4 = transforms[totalIndex + gaps[i] + gaps[i + 1]];
            }
            transforms[totalIndex + j] = interpolateFT(u, B, ft1, 
                                                    ft2, ft3, ft4);
        }
        totalIndex += gaps[i];
    }
}

/* Create the interpolated keyframe given a float u from 0 to 1, the matrix 
 * B used in the cardinal curve functions, and keyframes K_{i-1}, K_i, K_{i+1},
 * and K_{i+2}
 */
FrameTransform Frames::interpolateFT(double u, Matrix4d B, FrameTransform ft1, 
            FrameTransform ft2, FrameTransform ft3, FrameTransform ft4) {
    FrameTransform inter;
    Vector4d realVec;
    realVec << ft1.rotation.getReal(), ft2.rotation.getReal(), 
            ft3.rotation.getReal(), ft4.rotation.getReal();
    float real = interpolateVal(u, B, realVec);
    Vector3d imag;
    for (int i = 0; i < 3; i++) {
        Vector4d scaleVec;
        Vector4d tranVec;
        Vector4d imagVec;
        scaleVec << ft1.scale[i], ft2.scale[i], ft3.scale[i], ft4.scale[i];
        tranVec << ft1.translation[i], ft2.translation[i], ft3.translation[i], 
                    ft4.translation[i];
        imagVec << ft1.rotation.getImage()[i], ft2.rotation.getImage()[i], 
                    ft3.rotation.getImage()[i], ft4.rotation.getImage()[i];
        inter.scale[i] = interpolateVal(u, B, scaleVec);
        inter.translation[i] = interpolateVal(u, B, tranVec);
        imag[i] = interpolateVal(u, B, imagVec);
    }
    inter.rotation = Quat(real, imag);
    inter.rotation.normalize();
    return inter;
}

/* Returns the interpolated value between p_i and p_{i+1} given a float between 
 * 0 and 1, the B matrix, and a p vector consisting of p_{i-1}, p_{i}, p_{i+1}, 
 * p_{i+2}
 */
double Frames::interpolateVal(double u, Matrix4d B, Vector4d p) {
    RowVector4d vec;
    vec << 1, u, pow(u, 2), pow(u,3);
    VectorXd result = (vec * B * p);
    return result[0];
}

