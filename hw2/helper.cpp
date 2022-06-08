#include "helper.h"

/* Takes in a .obj file name and reads in vertices and faces into input vectors
 */ 
void readFile(string name, vector<Vertex>& vertices, vector<Normal>& normals, 
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
            float f1, f2, f3;
            iss >> f1 >> f2 >> f3;
            Vertex v(f1, f2, f3);
            vertices.push_back(v);
        }
        else if  (dtype.compare("vn") == 0) {
            // Read in three floats
            float f1, f2, f3;
            iss >> f1 >> f2 >> f3;
            Normal vn(f1, f2, f3);
            vn.normalize();
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
    return pair<int, int>(stoi(token1), stoi(str));
}

/* Takes a fstream of a file and reads in lines of the format 
 * "object1 object1_filename.obj" until an empty line is reached. The object
 * information of object1 is loaded in from "object1_filename.obj"
 * 
 * objVertices and objFaces map object labels to corresponding vertices and 
 * faces respectively
 */
void readObj(fstream& myfile, string path, 
            unordered_map<string, vector<Vertex>>& objVertices,
            unordered_map<string, vector<Normal>>& objNormals,
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
        vector<Normal> normals;
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

/* Computes the projection matrix based on the read-in perspective values.
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
                    unordered_map<string, vector<Normal>> objNormals,
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
        vector<MatTransform> matrices;
        Object obj(label, counts[label], objVertices[label], objNormals[label],
             objFaces[label]);
        getline(myfile, line);
        // Read in object properties and object transformations
        while (line.compare("") != 0) {
            istringstream iss(line);
            string dtype; 
            iss >> dtype;
            if (dtype.compare("shininess") == 0) {
                iss >> obj.phong;
            }
            else if (dtype.compare("ambient") == 0) {
                iss >> obj.ambient.r >> obj.ambient.g >> obj.ambient.b;
            }
            else if (dtype.compare("diffuse") == 0) {
                iss >> obj.diffuse.r >> obj.diffuse.g >> obj.diffuse.b;
            }
            else if (dtype.compare("specular") == 0) {
                iss >> obj.specular.r >> obj.specular.g >> obj.specular.b;
            }
            else {
                float x, y, z;
                iss >> x >> y >> z;
                Matrix4d m;
                MatTransform tr; 
                if (dtype.compare("t") == 0) {
                    m << 1, 0, 0, x,   
                        0, 1, 0, y,    
                        0, 0, 1, z,  
                        0, 0, 0, 1; 
                }
                else if (dtype.compare("r") == 0) {
                    // Read in the angle in radians
                    float theta;
                    iss >> theta;
                    m << getRotationMatrix(x, y, z, theta);
                }
                else if (dtype.compare("s") == 0){
                    m << x, 0, 0, 0,   
                        0, y, 0, 0,    
                        0, 0, z, 0,  
                        0, 0, 0, 1; 
                }
                tr.mat = m;
                tr.mtype = dtype;
                matrices.push_back(tr);
            }
            getline(myfile, line);
        }
        obj.matrices = matrices;
        Matrix4d tmatrix = obj.getTransformMatrix(true);
        // Transform vertices by geometric transformations to get the 
        // correct coordinates in world space
        for (int i = 1; i < obj.vertices.size(); i++) {
            Vertex vertex = objVertices[label][i];
            Vector4d vec;
            vec << vertex.x, vertex.y, vertex.z, 1;
            Vector4d prod = tmatrix * vec;
            Vertex newVertex(prod[0], prod[1], prod[2]);
            obj.vertices[i] = newVertex;
        }
        objects.push_back(obj);
    }
}

/* Parses a scene description file and puts the information into appropriate 
 * data structures
 */
void readScene(fstream& myfile, string path, Camera& c, Perspective& p, 
                vector<Object>& objects,vector<Light>& lights, 
                unordered_map<string, vector<Vertex>>& objVertices, 
                unordered_map<string, vector<Normal>>& objNormals, 
                unordered_map<string, vector<Face>>& objFaces) {
    string line;
    getline(myfile, line);
    while (line.compare("") != 0) {
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
    // Read in the lights
    getline(myfile, line);
    while (line.compare("objects:") != 0) { 
        istringstream iss(line);
        string str; 
        iss >> str;
        if (str.compare("light") == 0) {
            Light light;
            iss >> light.x >> light.y >> light.z;
            iss >> str;
            Color c;
            iss >> c.r >> c.g >> c.b;
            light.c = c;
            iss >> str;
            iss >> light.atten;
            lights.push_back(light);
        }
        getline(myfile, line);
    }
    readObj(myfile, path, objVertices, objNormals, objFaces);
}

/* Given the axis direction and axis-angle, computes the corresponding rotation
 * matrix. Normalizes the axis direction before computing the matrix
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

/* Returns the overall transformation matrix by multiplying together the 
 * individual geometric transformations 
 * 
 * translate is true if we include the translation matrices, false if not
 */ 
Matrix4d Object::getTransformMatrix(bool translate) {
    Matrix4d prod = Matrix4d::Identity();
    for (int i = 0; i < matrices.size(); i++) {
        if (translate || matrices[i].mtype.compare("t") != 0) {
            prod = matrices[i].mat * prod;
        }
    }
    return prod;
}

/* Normalizes the stored normal
 */
void Normal::normalize() {
    Vector3d vec;
    vec << x, y, z;
    vec.normalize();
    x = vec[0];
    y = vec[1];
    z = vec[2];
}

/* Transforms the normal using the transpose of the inverse of the object 
 * transformation matrix. Normalizes the transformed normal before returning
 * it
 */
Vector3d Normal::transformNormal(Matrix4d tmatrix) {
    Vector4d vec;
    vec << x, y, z, 1;
    Matrix4d inv = tmatrix.inverse();
    Matrix4d trans = inv.transpose();
    Vector4d prod = trans * vec;
    Vector3d surfNorm = prod.head(3);
    surfNorm.normalize();
    return surfNorm; 
}

/* Converts world coordinates to NDC using the camera transformation matrix
 * and perspective projection matrix
 */
Vector3d convertWorldToNDC(Vertex v, Object obj, Camera c, Perspective p) {
    Vector4d vec;
    vec << v.x, v.y, v.z, 1;
    Vector4d nvec =  c.getWorldToCamera() * vec;
    Vector4d ndc_vec = p.getProjection() * nvec;
    Vector4d scaled_vec = ndc_vec / ndc_vec[3];
    return scaled_vec.head(3);
}

/* Convert Cartesian NDC coordinates to a yres by xres pixel grid. Ignore the
 * z coordinates in the mapping
 */
pair<int, int> convertNDCToScreen(Vector3d coords, int xres, int yres) {
    float nx = xres / 2;
    float ny = yres / 2;
    int x_coord = (coords[0] + 1.0) * nx;
    int y_coord = (-coords[1] + 1.0) * ny;
    pair<int, int> new_pair(x_coord, y_coord);
    return new_pair;
}

/* Convertes vertex position to a vector for calculations
 */
Vector3d Vertex::getPos() {
    Vector3d pos;
    pos << x, y, z;
    return pos;
}

/* Convertes rgb values to a vector for calculations
 */
Vector3d Color::getVector() {
    Vector3d color;
    color << r, g, b;
    return color;
}

/* Convertes light position to a vector for calculations
 */
Vector3d Light::getPosVector() {
    Vector3d pos;
    pos << x, y, z;
    return pos;
}

/* Convertes camera position to a vector for calculations
 */
Vector3d Camera::getPosVector() {
    Vector3d pos;
    pos << posx, posy, posz;
    return pos;
}

/* Initializes an image object with a specified x-resolution and y-resolution.
 * Contains a grid of rgb values to output as well as depth buffer values
 */
Image::Image(int xres, int yres) {
    this->xres = xres;
    this->yres = yres;
    for (int j = 0; j < yres; j++) {
        vector<Color> row(xres);
        grid.push_back(row);
        vector<double> brow(xres, DBL_MAX);
        buffer.push_back(brow);
    }
}

/* Fills the given position with a color 
 */
void Image::fillGrid(int x, int y, Color c) {
    grid[y][x] = c;
}

/* Fills the given position in the buffer grid with a double 
 */
void Image::fillBuffer(int x, int y, double val) {
    buffer[y][x] = val;
}

/* Outputs the image using the stored grid color values
 */
void Image::outputGrid() {
    cout <<  "P3" << endl; 
    cout << xres << " " << yres << endl;
    cout << 255 << endl;
    for (int y = 0; y < yres; y++) {
        for (int x = 0; x < xres; x++) {
            Color c = grid[y][x];
            int red = c.r * 255.0;
            int blue = c.b * 255.0;
            int green = c.g * 255.0;
            cout << red << " " << green << " " << blue << endl;
        }
    }
}
