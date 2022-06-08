#include <Eigen/Dense>

using namespace Eigen;

class Quat {
    /* The real component */
    float real;
    /* The imaginary components */  
    Vector3d imag;
    public:
        Quat(const Quat &other) : real(other.real), imag(other.imag) {};
        Quat() : real(1), imag(Vector3d::Zero()) {};
        Quat(float real, Vector3d imag) : real(real), imag(imag) {};
        Quat(float rotation[]);
        Quat & operator*=(const Quat &sec);
        float getReal();
        Vector3d getImage();
        /* Normalizes the quaternion */
        void normalize();
        /* Converts the quaternion to a rotation matrix */
        Matrix4d getRotationMatrix();
};

Quat operator*(const Quat &first, const Quat &sec);