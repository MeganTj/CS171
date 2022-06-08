#include "quat.h"

/* Multiply two quaternions and return the result
 */
Quat operator*(const Quat &first, const Quat &sec) {
    Quat temp{first};
    temp *= sec;
    return temp;
}

/* Multiply the first quaternion by the second, saving the result into the first
 */
Quat & Quat::operator*=(const Quat &sec) {
    float newReal = real * sec.real - imag.dot(sec.imag);
    Vector3d newImag = real * sec.imag + sec.real * imag + imag.cross(sec.imag);
    real = newReal;
    imag = newImag;
    return *this;
}

/* Converts a quaternion into a rotation matrix
 */
Matrix4d Quat::getRotationMatrix() {
    Matrix4d mat;
    float A = 1- 2 * pow(imag[1], 2) - 2 * pow(imag[2], 2);
    float B = 2 * (imag[0] * imag[1] - imag[2] * real);
    float C = 2 * (imag[0] * imag[2] + imag[1] * real);
    float D = 2 * (imag[0] * imag[1] + imag[2] * real);
    float E = 1- 2 * pow(imag[0], 2) - 2 * pow(imag[2], 2);
    float F = 2 * (imag[1] * imag[2] - imag[0] * real);
    float G = 2 * (imag[0] * imag[2] - imag[1] * real);
    float H = 2 * (imag[1] * imag[2] + imag[0] * real);
    float I = 1- 2 * pow(imag[0], 2) - 2 * pow(imag[1], 2);
    mat << A, B, C, 0, 
           D, E, F, 0, 
           G, H, I, 0, 
           0, 0, 0, 1;
    return mat;
}

/* Compute the rotation quaternion based on the starting NDC coordinates
 * and the current NDC coordinates
 */
Quat Quat::computeRotationQuat(Vector3d scoords, Vector3d ccoords) {
    float input = scoords.dot(ccoords) / (scoords.norm() * ccoords.norm());
    float theta = acos(std::min(input, (float) 1.0));
    Vector3d uvec = scoords.cross(ccoords);
    uvec.normalize();
    // Convert rotation angle and rotation axis to quaternion
    Quat quat;
    quat.real = cos(theta / 2);
    if (theta == 0) {
        uvec[0] = 1;
        uvec[1] = 1;
        uvec[2] = 1;
    }
    quat.imag[0] = sin(theta / 2) * uvec[0]; 
    quat.imag[1] = sin(theta / 2) * uvec[1]; 
    quat.imag[2] = sin(theta / 2) * uvec[2]; 
    return quat;
}
