#include "parsing.h"

/* Takes in a .obj file name and reads in vertices and faces into input vectors
 */
Object readObj(string name) {
    fstream myfile("keyframes/" + name);
    string line;
    getline(myfile, line);
    Object obj;
    while (!myfile.eof())
    {
        istringstream iss(line);
        // Check first character
        char dtype;
        iss >> dtype;
        if (dtype == 'v')
        {
            // Read in three floats
            float f1, f2, f3;
            iss >> f1 >> f2 >> f3;
            Vertex v(f1, f2, f3);
            obj.vertices.push_back(v);
        }
        else if (dtype == 'f')
        {
            // Read in three ints
            int i1, i2, i3;
            iss >> i1 >> i2 >> i3;
            Face f(i1, i2, i3);
            obj.faces.push_back(f);
        }
        getline(myfile, line);
    }
    myfile.close();
    return obj;
}

/* Produce a smooth interpolation across keyframes using Catmull-Rom splines.
 * Modifies the passed in vector to contain Object's for non-keyframe
 * frames. 
 */
void interpolateObjects(vector<Object>& objects)
{
    int gap = 5;
    /* Create the B matrix */
    Matrix4d B;
    B << 0, 2, 0, 0,
        -1, 0, 1, 0,
        2, -5, 4, -1,
        -1, 3, -3, 1;
    B *= 0.5;
    // cout << B << endl;
    for (int i = 0; i < objects.size() - 1; i += gap)
    {
        Object obj2 = objects[i];
        Object obj3 = objects[i + gap];
        for (int j = 1; j < gap; j++)
        {
            double u = (double)j / (double)gap;
            Object obj1;
            Object obj4;
            if (i == 0)
            {
                obj1 = objects[0];
                obj4 = objects[i + 2 * gap];
            }
            else if (i == 15)
            {
                obj1 = objects[i - gap];
                obj4 = objects[20];
            }
            else
            {
                obj1 = objects[i - gap];
                obj4 = objects[i + 2 * gap];
            }
            objects[i + j] = interpolateObj(u, B, obj1, obj2, obj3, obj4);
        }
    }
}

/* Create the interpolated frame given a float u from 0 to 1, the matrix 
 * B used in the cardinal curve functions, and keyframe K_{i-1}, K_i, K_{i+1},
 * and K_{i+2}
 */
Object interpolateObj(double u, Matrix4d B, Object obj1, Object obj2, 
                    Object obj3, Object obj4) {
    Object inter;
    for (int i = 0; i < obj1.vertices.size(); i++)
    {
        Vertex newVec;
        Vertex v1 = obj1.vertices[i];
        Vertex v2 = obj2.vertices[i];
        Vertex v3 = obj3.vertices[i];
        Vertex v4 = obj4.vertices[i];
        Vector4d xVec;
        xVec << v1.x, v2.x, v3.x, v4.x;
        newVec.x = interpolateVal(u, B, xVec);
        Vector4d yVec;
        yVec << v1.y, v2.y, v3.y, v4.y;
        newVec.y = interpolateVal(u, B, yVec);
        Vector4d zVec;
        zVec << v1.z, v2.z, v3.z, v4.z;
        newVec.z = interpolateVal(u, B, zVec);
        inter.vertices.push_back(newVec);
    }
    inter.faces = obj1.faces;
    return inter;
}

/* Returns the interpolated value between p_i and p_{i+1} given a float between 
 * 0 and 1, the B matrix, and a p vector consisting of p_{i-1}, p_{i}, p_{i+1}, 
 * p_{i+2}
 */
double interpolateVal(double u, Matrix4d B, Vector4d p) {
    RowVector4d vec;
    vec << 1, u, pow(u, 2), pow(u, 3);
    VectorXd result = (vec * B * p);
    return result[0];
}

/* Outputs the vertices and faces of an object to a .obj file with the passed
 * in name, stored in the folder named output
 */
void Object::outputObj(string name) {
    ofstream outputFile("output/" + name);
    for (Vertex v : vertices) {
        outputFile << "v " << v.x << " " << v.y << " " << v.z << endl;
    }
    for (Face f : faces) {
        outputFile << "f " << f.v1 << " " << f.v2 << " " << f.v3 << endl;
    }
}

int main(int argc, char* argv[]) {
    // We know the number of frames ahead of time
    vector<Object> objects(21);
    string pref("bunny");
    string suffix(".obj");
    // Read in the obj files
    for (int i = 0; i <= objects.size(); i+= 5) {
        if (i < 10) {
            objects[i] = readObj(pref + "0" + to_string(i) + suffix);
        }
        else {
            objects[i] = readObj(pref + to_string(i) + suffix);
        }
    }
    // Output the interpolated keyframes
    interpolateObjects(objects);
    for (int i = 0; i < objects.size() - 1; i+= 5) {
        for (int j = 1; j < 5; j++) {
            int index = i + j;
            if (index < 10) {
                objects[index].outputObj(pref + "0" + to_string(index) + 
                                        suffix);
            }
            else {
                objects[index].outputObj(pref + to_string(index) + suffix);
            }
        }
    }
}

