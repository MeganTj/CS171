#include "part1.h"

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
    myfile.close();
}

/* Prints to standard output the vertices and faces information stored in each
 * .obj file
 */
void printFile(string name, vector<Vertex>& vertices,  vector<Face>& faces) {
    cout << "obj_" << name << ":" << endl;
    cout << endl;
    for (int i = 1; i < vertices.size(); i++) {
        Vertex v = vertices[i];
        printf("v %.3f %.3f %.3f\n", v.x, v.y, v.z);
    }
    for (Face f : faces) {
        cout << "f " << f.v1 << " " << f.v2 <<  " " << f.v3 << endl;
    }
}

/* Assumes that each input is the name of a .obj file
 */
int main(int argc, char** argv) {
    for (int i = 1; i < argc; i++) {
        // Read vertices and faces from obj files
        vector<Vertex> vertices;
        vector<Face> faces;
        Vertex empty;
        // In order to account for 1-indexing
        vertices.push_back(empty);
        readFile(argv[i], vertices, faces);
        printFile(argv[i], vertices, faces);
        cout << endl;
    }
    return 0;
}