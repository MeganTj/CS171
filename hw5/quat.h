#include <Eigen/Dense>

using namespace Eigen;

class Quat {
    /* The real component */
    float real;
    /* The imaginary components */  
    Vector3d imag;
    public:
        Quat & operator*=(const Quat &sec);
        /* Constructs a unit quaternion */
        Quat(const Quat &other) : real(other.real), imag(other.imag) {};
        Quat() : real(1), imag(Vector3d::Zero()) {};
        /* Converts the quaternion to a rotation matrix */
        Matrix4d getRotationMatrix();
        /* Compute the rotation quaternion based on the start and end NDC
        coordinates*/
        static Quat computeRotationQuat(Vector3d scoord, Vector3d ccoords);
};

Quat operator*(const Quat &first, const Quat &sec);