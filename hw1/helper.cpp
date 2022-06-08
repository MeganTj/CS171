#include "helper.h"

/* Takes in a .obj file name and reads in vertices and faces into input vectors
 */ 
void readFile(string name, vector<Vertex>& vertices,  vector<Face>& faces) {
    fstream myfile("data/" + name);
    string line; 
    getline(myfile, line);
    while (! myfile.eof()) {
        istringstream iss(line);
        // Check first character 
        char dtype; 
        iss >> dtype;
        if  (dtype == 'v') {
            // Read in three floats
            float f1, f2, f3;
            iss >> f1 >> f2 >> f3;
            Vertex v(f1, f2, f3);
            vertices.push_back(v);
        }
        else if (dtype == 'f') {
            // Read in three ints
            int i1, i2, i3;
            iss >> i1 >> i2 >> i3;
            Face f(i1, i2, i3);
            faces.push_back(f);
        }
        getline(myfile, line);
    }
}

/* Takes a fstream of a file and reads in lines of the format 
 * "object1 object1_filename.obj" until an empty line is reached. The object
 * information of object1 is loaded in from "object1_filename.obj"
 * 
 * objVertices and objFaces map object labels to corresponding vertices and 
 * faces respectively
 */
void readObj(fstream& myfile, unordered_map<string, vector<Vertex>>& objVertices,
             unordered_map<string, vector<Face>>& objFaces) {
    string line; 
    getline(myfile, line);
    while (line.compare("") != 0) {
        istringstream iss(line);
        // Get the object label
        string label; 
        iss >> label;
        string fname;
        iss >> fname;

        vector<Vertex> vertices;
        vector<Face> faces;
        Vertex empty;
        // In order to account for 1-indexing
        vertices.push_back(empty);
        readFile(fname, vertices, faces);
        objVertices[label] = vertices;
        objFaces[label] = faces;
        getline(myfile, line);
    }
}

/* 
 * For every set of transformations, creates a new object with the appropriate 
 * vertices and faces as well as the transformation matrices to be applied to 
 * the object. Appends individual objects to the objects vector, passed in 
 * as input
 * 
 * objVertices and objFaces map object labels to corresponding vertices and 
 * faces respectively
 */
void readTransforms(fstream& myfile, vector<Object>& objects, 
                    unordered_map<string, vector<Vertex>> objVertices, 
                    unordered_map<string, vector<Face>> objFaces) {
    string line; 
    unordered_map<string, int> counts;
    while (! myfile.eof()) {
        getline(myfile, line);
        // Read the object label;
        istringstream iss(line);
        string label;
        iss >> label;
        counts[label]++;
        // Construct a new Object 
        vector<Matrix4d> matrices;
        Object obj(label, counts[label], objVertices[label], objFaces[label]);
        // Read in each transform
        getline(myfile, line);
        while (line.compare("") != 0) {
            istringstream iss(line);
            char dtype; 
            iss >> dtype;
            float x, y, z;
            iss >> x >> y >> z;
            Matrix4d m;
            if (dtype == 't') {
                m << 1, 0, 0, x,   
                    0, 1, 0, y,    
                    0, 0, 1, z,  
                    0, 0, 0, 1; 
            }
            else if (dtype == 'r') {
                // Read in the angle in radians
                float theta;
                iss >> theta;
                m << getRotationMatrix(x, y, z, theta);
            }
            else {
                m << x, 0, 0, 0,   
                    0, y, 0, 0,    
                    0, 0, z, 0,  
                    0, 0, 0, 1; 
            }
            matrices.push_back(m);
            getline(myfile, line);
        }
        obj.matrices = matrices;
        objects.push_back(obj);
    }
}

/* Parses a scene description file, given its filestream, and puts data into 
 * the objects vector. Also reads in camera and perspective matrix values
 * 
 * objVertices and objFaces map object labels to corresponding vertices and 
 * faces respectively
 */
void readScene(fstream& myfile, Camera& c, Perspective& p, 
                vector<Object>& objects, 
                unordered_map<string, vector<Vertex>>& objVertices, 
                unordered_map<string, vector<Face>>& objFaces) {
    string line;
    getline(myfile, line);
    while (line.compare("objects:") != 0) {
        istringstream iss(line);
        if (line.compare("camera:") != 0) {
            string begin; 
            iss >> begin;
            if (begin.compare("position") == 0) {
                iss >> c.posx >> c.posy >> c.posz;
            }
            else if (begin.compare("orientation") == 0) {
                iss >> c.orx >> c.ory >> c.orz >> c.theta;
            }
            else {
                float val;
                iss >> val;
                if (begin.compare("near") == 0) {
                    p.n = val;
                }
                else if (begin.compare("far") == 0) {
                    p.f = val;
                }
                else if (begin.compare("left") == 0) {
                    p.l = val;
                }
                else if (begin.compare("right") == 0) {
                    p.r = val;
                }
                else if (begin.compare("top") == 0){
                    p.t = val;
                }
                else if (begin.compare("bottom") == 0){
                    p.b = val;
                }
            }
        }
        getline(myfile, line);
    }
    readObj(myfile, objVertices, objFaces);
}

/* Computes the rotation matrix given the direction of the axis and rotation
 * angle. Importantly, normalizes the axis direction before computing the
 * matrix.
 */
Matrix4d getRotationMatrix(float x, float y, float z, float theta) {
    Vector3d vec;
    vec << x, y, z;
    vec.normalize();
    x = vec[0];
    y = vec[1];
    z = vec[2];
    float A = pow(x, 2) + (1 - pow(x, 2)) * cos(theta);
    float B = x * y * (1 - cos(theta)) - z * sin(theta);
    float C = x * z * (1 - cos(theta)) + y * sin(theta);
    float D = x * y * (1 - cos(theta)) + z * sin(theta);
    float E = pow(y, 2) + (1 - pow(y, 2)) * cos(theta);
    float F = y * z * (1 - cos(theta)) - x * sin(theta);
    float G = x * z * (1 - cos(theta)) - y * sin(theta);
    float H = y * z * (1 - cos(theta)) + x * sin(theta);
    float I = pow(z, 2) + (1 - pow(z, 2)) * cos(theta);
    Matrix4d m;
    m << A, B, C, 0,   
        D, E, F, 0,    
        G, H, I, 0,  
        0, 0, 0, 1;
    return m;
}

/* Compute the overall transformation for the camera C = TR and then return
 * C^(-1), which is what we apply to every point in world space.
 */
Matrix4d Camera::getWorldToCamera() {
    Matrix4d translation;
    translation << 1, 0, 0, posx,   
                    0, 1, 0, posy,    
                    0, 0, 1, posz,  
                    0, 0, 0, 1;   
    Matrix4d rotation;   
    rotation << getRotationMatrix(orx, ory, orz, theta);
    Matrix4d prod = translation * rotation;
    Matrix4d inv = prod.inverse();
    return inv;
}

/* Constructs the projection matrix using the read-in values
 */ 
Matrix4d Perspective::getProjection() {
    Matrix4d proj;
    float A = 2 * n / (r - l);
    float B = (r + l) / (r - l);
    float C = 2 * n / (t - b);
    float D = (t + b) / (t - b);
    float E = -(f + n) / (f - n);
    float F = -2 * f * n / (f - n);
    proj << A, 0, B, 0,
            0, C, D, 0,
            0, 0, E, F,
            0, 0, -1, 0;
    return proj;
}