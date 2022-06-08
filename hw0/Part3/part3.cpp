#include "part3.h"

/* Takes in a .obj file name and reads in vertices and faces into input vectors
 */ 
void readFile(string name, vector<Vertex>& vertices,  vector<Face>& faces) {
    fstream myfile(name);
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
void readObj(fstream& myfile, unordered_map<string, vector<Vertex>>& 
            objVertices, unordered_map<string, vector<Face>>& objFaces) {
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
 * vertices and faces as well as thetransformation matrices to be applied to 
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
        //Construct a new Object 
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
                // Normalize the passed in vector
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
            matrices.push_back(m);
            getline(myfile, line);
        }
        obj.matrices = matrices;
        objects.push_back(obj);
    }
}

/* Applies the transformations specified in the matrices attribute and prints
 * out the transformed vertices
 */ 
void Object::applyAndPrint() {
    cout << label << "_copy" << copyNum << endl;
    // Cumulate transformations by matrix multiplication
    Matrix4d prod = Matrix4d::Identity();
    for (int i = 0; i < matrices.size(); i++) {
        prod = matrices[i] * prod;
    }
    // Apply transformations and print out transformed vertices
    for (int i = 1; i < vertices.size(); i++) {
        Vertex v = vertices[i];
        Vector4d vec;
        vec << v.x, v.y, v.z, 1;
        Vector4d t = prod * vec;
        RowVector4d scaled = t / t[3];
        // Print out the x, y, z coordinate
        cout << scaled.head(3) << endl;
    }
}

/* Assumes that the name of a single .txt file is passed in as input
 */
int main(int argc, char** argv) {
    // Store vertices and faces so that we 
    // can copy them when needed
    unordered_map<string, vector<Vertex>> objVertices;
    unordered_map<string, vector<Face>> objFaces;
    fstream myfile(argv[1]);
    // Read in object information
    readObj(myfile, objVertices, objFaces);
    vector<Object> objects;
    // Read in transformations to the objects
    readTransforms(myfile, objects, objVertices, objFaces);
    myfile.close();
    // Print out the transformed vertices 
    for (Object obj : objects) {
        obj.applyAndPrint();
        cout << endl;
    }
    return 0;
}