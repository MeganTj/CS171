#include "parsing.h"

/* Takes in a .obj file name and reads in vertices and faces into input vectors
 */ 
void readFile(string name, vector<Triple>& vertices, vector<Triple>& normals, 
        vector<Face>& faces) {
    fstream myfile(name);
    string line; 
    getline(myfile, line);
    while (! myfile.eof()) {
        istringstream iss(line);
        // Check first character 
        string dtype; 
        iss >> dtype;
        if (dtype.compare("v") == 0) {
            // Read in three floats
            Triple v;
            iss >> v.x >> v.y >> v.z;
            vertices.push_back(v);
        }
        else if  (dtype.compare("vn") == 0) {
            // Read in three floats
            Triple vn;
            iss >> vn.x >> vn.y >> vn.z;
            normals.push_back(vn);
        }
        else if (dtype.compare("f") == 0) {
            // Read in three ints
            string pair1, pair2, pair3;
            iss >> pair1 >> pair2 >> pair3;

            Face f(getPair(pair1), getPair(pair2), getPair(pair3));
            faces.push_back(f);
        }
        getline(myfile, line);
    }
}

/* Takes in a string in the form of <int 1>//<int 2> and returns a pair 
 * consisting of int1, int 2.
 */
pair<int, int> getPair(string str) {
    string delimeter = "//";
    int pos = str.find(delimeter);
    string token1 = str.substr(0, pos);
    str.erase(0, pos + delimeter.length());
    return pair<int, int> (stoi(token1), stoi(str));
}

/* Takes a fstream of a file and reads in lines of the format 
 * "object1 object1_filename.obj" until an empty line is reached. The object
 * information of object1 is loaded in from "object1_filename.obj"
 * 
 * objVertices and objFaces map object labels to corresponding vertices and 
 * faces respectively
 */
void readObj(fstream& myfile, string path, 
            unordered_map<string, vector<Triple>>& objVertices,
            unordered_map<string, vector<Triple>>& objNormals,
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
        vector<Triple> vertices;
        vector<Triple> normals;
        vector<Face> faces;

        // In order to account for 1-indexing
        vertices.push_back({});
         // In order to account for 1-indexing
        normals.push_back({});
        string objFile = path.substr(0, path.find_last_of('/')) + "/" + fname;
        readFile(objFile, vertices, normals, faces);
        objVertices[label] = vertices;
        objNormals[label] = normals;
        objFaces[label] = faces;
        getline(myfile, line);
    }
}

/* Parses a scene description file and puts the information into appropriate 
 * camera, perspective, object, and light data structures. 
 */
void readScene(fstream& myfile, string path, Camera& c, Perspective& p, 
                vector<Object>& objects, vector<Point_Light>& lights, 
                unordered_map<string, vector<Triple>>& objVertices, 
                unordered_map<string, vector<Triple>>& objNormals, 
                unordered_map<string, vector<Face>>& objFaces) {
    string line;
    getline(myfile, line);
    while (line.compare("") != 0) {
        istringstream iss(line);
        if (line.compare("camera:") != 0) {
            string begin; 
            iss >> begin;
            if (begin.compare("position") == 0) {
                iss >> c.cam_position[0] >> c.cam_position[1] >> 
                        c.cam_position[2];
            }
            else if (begin.compare("orientation") == 0) {
                iss >> c.cam_orientation_axis[0] >> c.cam_orientation_axis[1] 
                    >> c.cam_orientation_axis[2] >> c.cam_orientation_angle;
            }
            else {
                float val;
                iss >> val;
                if (begin.compare("near") == 0) {
                    p.near_param = val;
                }
                else if (begin.compare("far") == 0) {
                    p.far_param = val;
                }
                else if (begin.compare("left") == 0) {
                    p.left_param = val;
                }
                else if (begin.compare("right") == 0) {
                    p.right_param = val;
                }
                else if (begin.compare("top") == 0){
                    p.top_param = val;
                }
                else if (begin.compare("bottom") == 0){
                    p.bottom_param = val;
                }
            }
        }
        getline(myfile, line);
    }
    // Read in the lights
    getline(myfile, line);
    while (line.compare("objects:") != 0) { 
        istringstream iss(line);
        string str; 
        iss >> str;
        if (str.compare("light") == 0) {
            Point_Light light;
            iss >> light.position[0] >> light.position[1] >> light.position[2];
            light.position[3] = 1;
            iss >> str;
            iss >> light.color[0] >> light.color[1] >> light.color[2];
            iss >> str;
            iss >> light.attenuation_k;
            lights.push_back(light);
        }
        getline(myfile, line);
    }
    readObj(myfile, path, objVertices, objNormals, objFaces);
}

/* 
 * For every set of transformations, creates a new object with the appropriate 
 * vertices and faces as well as thetransformation matrices to be applied to 
 * the object. Appends individual objects to the objects vector, passed in 
 * as input
 * 
 * objVertices and objFaces map object labels to corresponding vertices and 
 * faces respectively
 */
void readTransforms(fstream& myfile, vector<Object>& objects, 
                    unordered_map<string, vector<Triple>> objVertices, 
                    unordered_map<string, vector<Triple>> objNormals,
                    unordered_map<string, vector<Face>> objFaces) {
    string line; 
    unordered_map<string, int> counts;
    while (! myfile.eof()) {
        getline(myfile, line);
        // Read the object label;
        istringstream iss(line);
        string label;
        iss >> label;
        // Construct a new Object 
        vector<MatTransform> transforms;
        vector<Triple> allVertices = objVertices[label];
        vector<Triple> allNormals = objNormals[label];
        vector<Triple> orderedVertices;
        vector<Triple> orderedNormals;
        for (Face f : objFaces[label]) {
            orderedVertices.push_back(allVertices[f.v1.first]);
            orderedVertices.push_back(allVertices[f.v2.first]);
            orderedVertices.push_back(allVertices[f.v3.first]);
            orderedNormals.push_back(allNormals[f.v1.second]);
            orderedNormals.push_back(allNormals[f.v2.second]);
            orderedNormals.push_back(allNormals[f.v3.second]);
        }
        Object obj;
        obj.vertex_buffer = orderedVertices;
        obj.normal_buffer = orderedNormals;
        // Read in each transform
        getline(myfile, line);
        while (line.compare("") != 0) {
            istringstream iss(line);
            string dtype; 
            iss >> dtype;
            if (dtype.compare("shininess") == 0) {
                iss >> obj.shininess;
            }
            else if (dtype.compare("ambient") == 0) {
                iss >> obj.ambient_reflect[0] >> obj.ambient_reflect[1] >> 
                    obj.ambient_reflect[2];
            }
            else if (dtype.compare("diffuse") == 0) {
                iss >> obj.diffuse_reflect[0] >> obj.diffuse_reflect[1] >> 
                    obj.diffuse_reflect[2];
            }
            else if (dtype.compare("specular") == 0) {
                iss >> obj.specular_reflect[0] >> obj.specular_reflect[1] >> 
                    obj.specular_reflect[2];
            }
            else {
                float x, y, z;
                iss >> x >> y >> z;
                MatTransform tr; 
                if (dtype.compare("r") == 0) {
                    // Read in the angle in radians
                    float theta;
                    iss >> theta;
                    tr.rotation_angle = theta;
                    float norm = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
                    x /= norm;
                    y /= norm;
                    z /= norm;
                }
                tr.transform[0] = x;
                tr.transform[1] = y;
                tr.transform[2] = z;
                tr.mtype = dtype;
                transforms.push_back(tr);
            }
            getline(myfile, line);
        }
        obj.transforms = transforms;
        objects.push_back(obj);
    }
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
