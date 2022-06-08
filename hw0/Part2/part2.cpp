#include <iostream>
#include <fstream>
#include <sstream> 
#include <cmath>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

/* Takes the name of a file containing a list of translation, rotation, and  
 * scaling vectors, creates the corresponding transformation matrices, and 
 * outputs to standard output the inverse of the product of all matrices
 */
Matrix4d computeProd(char* name) {
    fstream myfile(name);
    string line; 
    getline(myfile, line);
    Matrix4d prod = Matrix4d::Identity();
    while (! myfile.eof()) {
        istringstream iss(line);
        // Check first character 
        char dtype; 
        iss >> dtype;
        // Read in three floats
        float x, y, z;
        iss >> x >> y >> z;
        Matrix4d m;
        if  (dtype == 't') {
            m << 1, 0, 0, x,   
                 0, 1, 0, y,    
                 0, 0, 1, z,  
                 0, 0, 0, 1; 
        }
        else if (dtype == 'r') {
            // Read in the angle in radians
            float theta;
            iss >> theta;
            float norm = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
            x /= norm;
            y /= norm;
            z /= norm;
            float A = pow(x, 2) + (1 - pow(x, 2)) * cos(theta);
            float B = x * y * (1 - cos(theta)) - z * sin(theta);
            float C = x * z * (1 - cos(theta)) + y * sin(theta);
            float D = x * y * (1 - cos(theta)) + z * sin(theta);
            float E = pow(y, 2) + (1 - pow(y, 2)) * cos(theta);
            float F = y * z * (1 - cos(theta)) - x * sin(theta);
            float G = x * z * (1 - cos(theta)) - y * sin(theta);
            float H = y * z * (1 - cos(theta)) + x * sin(theta);
            float I = pow(z, 2) + (1 - pow(z, 2)) * cos(theta);
            m << A, B, C, 0,   
                 D, E, F, 0,    
                 G, H, I, 0,  
                 0, 0, 0, 1;
        }
        else {
            m << x, 0, 0, 0,   
                 0, y, 0, 0,    
                 0, 0, z, 0,  
                 0, 0, 0, 1; 
        }
        prod = m * prod;
        getline(myfile, line);
    }
    myfile.close();
    return prod.inverse();
}

/* Takes in a single text file of translation, scaling,
 * rotation vectors 
 */
int main(int argc, char** argv) {
    Matrix4d result = computeProd(argv[1]);
    cout << result << endl;
    return 0;
}