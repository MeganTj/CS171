#include "quat.h"

/* Constructs a quaternion from a rotation array, with the last position
 * being the rotation angle 
 */
Quat::Quat(float rotation[]) {
    float theta = rotation[3];
    real = cos(theta / 2);
    imag[0] = sin(theta / 2) * rotation[0]; 
    imag[1] = sin(theta / 2) * rotation[1]; 
    imag[2] = sin(theta / 2) * rotation[2]; 
}

float Quat::getReal() {
    return real;
}

Vector3d Quat::getImage() {
    return imag;
}

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

/* Normalize a quaternion by dividing by its norm
 */
void Quat::normalize() {
    float norm = sqrt(pow(real, 2) + pow(imag[0], 2) + pow(imag[1], 2) + 
                pow(imag[2], 2));
    real /= norm;
    imag /= norm;
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