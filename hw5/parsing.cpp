#include "parsing.h"

/* Takes in a .obj file name and reads in vertices and faces into input vectors. 
 */ 
void readFile(string name, Mesh_Data& d) {
    vector<Vertex*> *vertices = new vector<Vertex*>();
    // To account for 1-indexing
    (*vertices).push_back({});
    vector<Face*> *faces = new vector<Face*>();
    fstream myfile(name);
    string line; 
    getline(myfile, line);
    while (! myfile.eof()) {
        istringstream iss(line);
        // Check first character 
        string dtype; 
        iss >> dtype;
        if (dtype.compare("v") == 0) {
            Vertex *v = new Vertex();
            iss >> v->x >> v->y >> v->z;
            (*vertices).push_back(v);
        }
        else if (dtype.compare("f") == 0) {
            // Read in three ints
            Face *f = new Face();
            iss >> f->idx1 >> f->idx2  >> f->idx3 ;
            (*faces).push_back(f);
        }
        getline(myfile, line);
    }
    d.faces = faces;
    d.vertices = vertices;
}

/* Computes the area-weighted normal of a vertex by taking the weighted sum
 * of the normals of the incident faces.
 */
Vec3f calc_vertex_normal(HEV *vertex) {
    Vec3f normal;

    HE* he = vertex->out; // get outgoing halfedge from given vertex
    Vec3f curr_normal = calc_weighted_normal(he);
    normal.x = curr_normal.x;
    normal.y = curr_normal.y;
    normal.z = curr_normal.z;
    he = he->flip->next;
    while(he != vertex->out) 
    {
        curr_normal = calc_weighted_normal(he);
        // accummulate onto our normal vector
        normal.x += curr_normal.x;
        normal.y += curr_normal.y;
        normal.z += curr_normal.z;
        // gives us the halfedge to the next adjacent vertex
        he = he->flip->next;
    }
    return normal;
}

/* Computes the weighted normal of the face associated with the passed in
 * halfedge.
 */
Vec3f calc_weighted_normal(HE* he) {
    // Calculate the normal
    HEF *f = he->face;
    HEV *v1 = f->edge->vertex;
    HEV *v2 = f->edge->next->vertex;
    HEV *v3 = f->edge->next->next->vertex;
    Vector3d v1_coords = Vector3d(v1->x, v1->y, v1->z);
    Vector3d v2_coords= Vector3d(v2->x, v2->y, v2->z);
    Vector3d v3_coords= Vector3d(v3->x, v3->y, v3->z);
    Vector3d face_normal = (v2_coords - v1_coords).cross(v3_coords - v1_coords);
    // Compute the area of the triangular face
    double face_area = 0.5 * face_normal.norm();
    Vec3f normal;
    normal.x = face_normal[0] * face_area;
    normal.y = face_normal[1] * face_area;
    normal.z = face_normal[2] * face_area;
    return normal;
}

/* Takes a fstream of a file and reads in lines of the format 
 * "object1 object1_filename.obj" until an empty line is reached. The object
 * information of object1 is loaded in from "object1_filename.obj". We also
 * assign indices to the vertices here to be later used in implicit fairing
 * 
 * objVertices and objFaces map object labels to corresponding halfedge 
 * vertices and faces respectively. Also read in the Mesh_Data of all objects
 */
void readObj(fstream& myfile, unordered_map<string, Mesh_Data>& objMesh, 
unordered_map<string, vector<HEV*>>& objVertices, unordered_map<string, 
vector<HEF*>>& objFaces) {
    string line; 
    getline(myfile, line);
    while (line.compare("") != 0) {
        istringstream iss(line);
        // Get the object label
        string label; 
        iss >> label;
        string fname;
        iss >> fname;
        Mesh_Data d;

        readFile(fname, d);
        vector<HEV*> *hevs = new vector<HEV*>();
        vector<HEF*> *hefs= new vector<HEF*>();
        vector<Vec3f> normals;
        build_HE(&d, hevs, hefs);
        // For all vertices, assign indices and calculate the normals
        for (int i = 1; i < (*hevs).size(); i++) {
            HEV* vertex = (*hevs)[i];
            vertex->index = i;
            vertex->normal = calc_vertex_normal(vertex);
        }
        objMesh[label] = d;
        objVertices[label] = *hevs;
        objFaces[label] = *hefs;
        getline(myfile, line);
    }
}

/* Parses a scene description file and puts the information into appropriate 
 * data structures
 */
void readScene(fstream& myfile, Camera& c, Perspective& p, vector<Object>& objects,
vector<Point_Light>& lights, unordered_map<string, Mesh_Data>& objMesh, 
unordered_map<string, vector<HEV*>>& objVertices, 
unordered_map<string, vector<HEF*>>& objFaces) {
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
    readObj(myfile, objMesh, objVertices, objFaces);
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
                    unordered_map<string, vector<HEV*>>& objVertices, 
                    unordered_map<string, vector<HEF*>>& objFaces) {
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
        vector<HEV*> allVertices = objVertices[label];
        vector<HEF*> allFaces = objFaces[label];
        vector<Vec3f> orderedVertices;
        vector<Vec3f> orderedNormals;
        for (HEF* f : allFaces) {
            HEV *v1 = f->edge->vertex;
            HEV *v2 = f->edge->next->vertex;
            HEV *v3 = f->edge->next->next->vertex;
            Vec3f v1_coords{(float) v1->x, (float) v1->y, (float) v1->z};
            Vec3f v2_coords{(float) v2->x, (float) v2->y, (float) v2->z};
            Vec3f v3_coords{(float) v3->x, (float) v3->y, (float) v3->z};
            orderedVertices.push_back(v1_coords);
            orderedVertices.push_back(v2_coords);
            orderedVertices.push_back(v3_coords);
            orderedNormals.push_back(v1->normal);
            orderedNormals.push_back(v2->normal);
            orderedNormals.push_back(v3->normal);
        }
        Object obj;
        obj.vertex_buffer = orderedVertices;
        obj.normal_buffer = orderedNormals;
        obj.he_vertices = allVertices;
        obj.he_faces = allFaces;
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