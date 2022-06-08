#include "helper.h"
#include "renderer.h"

/* Computes the f_{ij} function that is used in computing barycentric
 * coordinates
 */
float baryHelper(float xi, float yi, float xj, float yj, float x, float y)  {
    return (yi - yj) * x + (xj - xi) * y + xi *  yj - xj * yi;
}

/* Compute alpha using f_{ij}
 */ 
float getAlpha(float xa, float ya, float xb, float yb, float xc, float yc, 
                float x, float y) {
    return baryHelper(xb, yb, xc, yc, x, y) / baryHelper(xb, yb, xc, yc, xa, 
                                                                        ya);
}   

/* Compute beta using f_{ij}
 */ 
float getBeta(float xa, float ya, float xb, float yb, float xc, float yc, 
                float x, float y) {
    return baryHelper(xa, ya, xc, yc, x, y) / baryHelper(xa, ya, xc, yc, xb, 
                                                                        yb);
}   

/* Compute gamma using f_{ij}
 */ 
float getGamma(float xa, float ya, float xb, float yb, float xc, float yc, 
                float x, float y) {
    return baryHelper(xa, ya, xb, yb, x, y) / baryHelper(xa, ya, xb, yb, xc, 
                                                                        yc);
}  

/* Performs lighting calcuations in world space. Takes into account diffuse,
 * ambinet, and specular lighting
 */
Color lighting(Vertex vec, Normal n, Object obj, vector<Light> lights, 
                Camera cam,  Perspective p) {
    Vector3d pos = vec.getPos();
    Vector3d c_d = obj.diffuse.getVector();
    Vector3d c_a = obj.ambient.getVector();
    Vector3d c_s = obj.specular.getVector();
    Vector3d diffuse_sum = Vector3d::Zero();
    Vector3d specular_sum = Vector3d::Zero();
    Vector3d e_dir = cam.getPosVector() - pos; 
    // Get the transformation matrix without translations, since these don't
    // affect computation of the new normals
    Matrix4d tmatrix = obj.getTransformMatrix(false);
    e_dir.normalize();
    for (Light l : lights) {
        Vector3d l_dir = l.getPosVector() - pos;
        // Vector3d l_dir = l.getPosVector();
        double dist = l_dir.norm();
        l_dir.normalize();
        Vector3d l_c = l.c.getVector();
        // Incorporate attenuation
        double factor = (double) 1 / (1 + l.atten * pow(dist, 2));
        l_c *= factor;
        n.normalize();
        Vector3d normed = n.transformNormal(tmatrix);
        Vector3d l_diffuse = l_c * max((double) 0, normed.dot(l_dir));
        diffuse_sum += l_diffuse;
        Vector3d sum_dir = e_dir + l_dir;
        sum_dir.normalize();
        Vector3d l_specular = l_c * pow(max((double) 0, normed.dot(sum_dir)), 
                            obj.phong);
        specular_sum += l_specular;
    }
    Vector3d allOnes = Vector3d::Ones();
    Vector3d calc = c_a + diffuse_sum.cwiseProduct(c_d) + 
                        specular_sum.cwiseProduct(c_s); 
    // Color values can't exceed 1
    Vector3d color = allOnes.cwiseMin(calc);
    Color c(color);
    return c;
}

/* Rasterizes the face using NDC coordinates and the color at each vertex
 */
void rasterColoredTriangle(Vector3d ndc_a, Vector3d ndc_b, Vector3d ndc_c, 
                        Color c_a, Color c_b, Color c_c, Image& img) {
    // Incorporate backface culling by not considering triangles that face
    // away from us 
    Vector3d cross = (ndc_c - ndc_b).cross(ndc_a - ndc_b);
    if (cross[2] < 0) {
        return;
    }
    pair<int, int> a_coords = convertNDCToScreen(ndc_a, img.xres, img.yres);
    pair<int, int> b_coords = convertNDCToScreen(ndc_b, img.xres, img.yres);
    pair<int, int> c_coords = convertNDCToScreen(ndc_c, img.xres, img.yres);
    int x_a = a_coords.first;
    int y_a = a_coords.second;
    int x_b = b_coords.first;
    int y_b = b_coords.second;
    int x_c = c_coords.first;
    int y_c = c_coords.second;
    int x_min = min(min(x_a, x_b), x_c);
    int x_max = max(max(x_a, x_b), x_c);
    int y_min = min(min(y_a, y_b), y_c);
    int y_max = max(max(y_a, y_b), y_c);
    for (int x = x_min; x <= x_max; x++) {
        for (int y = y_min; y <= y_max; y++) {
            float alpha = getAlpha(x_a, y_a, x_b, y_b, x_c, y_c, x, y);
            float beta = getBeta(x_a, y_a, x_b, y_b, x_c, y_c, x, y);
            float gamma = getGamma(x_a, y_a, x_b, y_b, x_c, y_c, x, y);
            // Check that the point is within the face
            if (alpha >= 0 && alpha <= 1 && beta >= 0 && beta <= 1 &&
            gamma >= 0 && gamma <= 1 ) {
                float ndc_x = alpha * ndc_a[0] + beta * ndc_b[0] + 
                                gamma * ndc_c[0];
                float ndc_y = alpha * ndc_a[1] + beta * ndc_b[1] + 
                                gamma * ndc_c[1];
                float ndc_z = alpha * ndc_a[2] + beta * ndc_b[2] + 
                                gamma * ndc_c[2];
                // Check that the point lies within the NDC cube, and that 
                // there is no other point in front of it
                if (abs(ndc_x) <= 1 && abs(ndc_y) <= 1 &&
                    abs(ndc_z) <= 1 && ndc_z <= img.buffer[y][x]) {
                    img.fillBuffer(x, y, ndc_z);
                    float red = alpha * c_a.r + beta * c_b.r + gamma * c_c.r;
                    float green = alpha * c_a.g + beta * c_b.g + gamma * c_c.g;
                    float blue = alpha * c_a.b + beta * c_b.b + gamma * c_c.b;
                    Color c(red, green, blue);
                    img.fillGrid(x, y, c);
                }
            }
        }
    }
}

/* Implements per-vertex shading. Interpolates colors and ndc coordinates
 * of the passed in vertices. Modifies the passed-in Image object.
 */
void gouraudShading(Vertex a, Vertex b, Vertex c, Normal n_a, Normal n_b, 
                    Normal n_c, Object obj, vector<Light> lights, Camera cam, 
                    Perspective p, Image& img) {
    Color c_a = lighting(a, n_a, obj, lights, cam, p);
    Color c_b = lighting(b, n_b, obj, lights, cam, p);
    Color c_c = lighting(c, n_c, obj, lights, cam, p);
    Vector3d ndc_a = convertWorldToNDC(a, obj, cam, p);
    Vector3d ndc_b = convertWorldToNDC(b, obj, cam, p);
    Vector3d ndc_c = convertWorldToNDC(c, obj, cam, p);
    rasterColoredTriangle(ndc_a, ndc_b, ndc_c, c_a, c_b, c_c, img);
}

/* Implements per-pixel shading. Computes lighting based on interpolated 
 * vertex positions and normals.
 */ 
void phongShading(Vertex a, Vertex b, Vertex c, Normal n_a, Normal n_b, 
                Normal n_c, Object obj, vector<Light> lights, Camera cam, 
                Perspective p, Image& img) {
    Vector3d ndc_a = convertWorldToNDC(a, obj, cam, p);
    Vector3d ndc_b = convertWorldToNDC(b, obj, cam, p);
    Vector3d ndc_c = convertWorldToNDC(c, obj, cam, p);
    // Incorporate backface culling by not considering triangles that face
    // away from us 
    Vector3d cross = (ndc_c - ndc_b).cross(ndc_a - ndc_b);
    if (cross[2] < 0) {
        return;
    }
    pair<int, int> a_coords = convertNDCToScreen(ndc_a, img.xres, img.yres);
    pair<int, int> b_coords = convertNDCToScreen(ndc_b, img.xres, img.yres);
    pair<int, int> c_coords = convertNDCToScreen(ndc_c, img.xres, img.yres);
    int x_a = a_coords.first;
    int y_a = a_coords.second;
    int x_b = b_coords.first;
    int y_b = b_coords.second;
    int x_c = c_coords.first;
    int y_c = c_coords.second;
    int x_min = min(min(x_a, x_b), x_c);
    int x_max = max(max(x_a, x_b), x_c);
    int y_min = min(min(y_a, y_b), y_c);
    int y_max = max(max(y_a, y_b), y_c);
    for (int x = x_min; x <= x_max; x++) {
        for (int y = y_min; y <= y_max; y++) {
            float alpha = getAlpha(x_a, y_a, x_b, y_b, x_c, y_c, x, y);
            float beta = getBeta(x_a, y_a, x_b, y_b, x_c, y_c, x, y);
            float gamma = getGamma(x_a, y_a, x_b, y_b, x_c, y_c, x, y);
            // Check that the point is within the face
            if (alpha >= 0 && alpha <= 1 && beta >= 0 && beta <= 1 &&
            gamma >= 0 && gamma <= 1) {
                float ndc_x = alpha * ndc_a[0] + beta * ndc_b[0] + 
                            gamma * ndc_c[0];
                float ndc_y = alpha * ndc_a[1] + beta * ndc_b[1] + 
                            gamma * ndc_c[1];
                float ndc_z = alpha * ndc_a[2] + beta * ndc_b[2] + 
                            gamma * ndc_c[2];
                // Check that the point lies within the NDC cube, and that 
                // there is no other point in front of it
                if (abs(ndc_x) <= 1 && abs(ndc_y) <= 1 &&
                    abs(ndc_z) && ndc_z <= img.buffer[y][x]) {
                    img.fillBuffer(x, y, ndc_z);
                    n_a.normalize();
                    n_b.normalize();
                    n_c.normalize();
                    float n_x = alpha * n_a.x + beta * n_b.x + gamma * n_c.x;
                    float n_y = alpha * n_a.y + beta * n_b.y + gamma * n_c.y;
                    float n_z = alpha * n_a.z + beta * n_b.z + gamma * n_c.z;
                    float v_x = alpha * a.x + beta * b.x + gamma * c.x;
                    float v_y = alpha * a.y + beta * b.y + gamma * c.y;
                    float v_z = alpha * a.z + beta * b.z + gamma * c.z;
                    Normal normal(n_x, n_y, n_z);
                    Vertex vertex(v_x, v_y, v_z);
                    Color c = lighting(vertex, normal, obj, lights, cam, p);
                    img.fillGrid(x, y, c);
                }
            }
        }
    }
}


int main(int argc, char** argv) {
    // Store vertices and faces so that we 
    // can copy them when needed
    unordered_map<string, vector<Vertex>> objVertices;
    unordered_map<string, vector<Normal>> objNormals;
    unordered_map<string, vector<Face>> objFaces;
    string path = string(argv[1]);
    fstream myfile(path);
    vector<Object> objects;
    vector<Light> lights;
    Camera cam;
    Perspective p;
    // Read in the scene description
    readScene(myfile, path, cam, p, objects, lights, objVertices, objNormals, 
            objFaces); 
    // Read in transformations to the objects
    readTransforms(myfile, objects, objVertices, objNormals, objFaces);
    myfile.close();
    int xres = atoi(argv[2]);
    int yres = atoi(argv[3]);
    int mode = atoi(argv[4]);
    Image img(xres, yres);
    for (Object obj : objects) {
        for (Face f : obj.faces) {
            Vertex a = obj.vertices[f.v1.first];
            Vertex b = obj.vertices[f.v2.first];
            Vertex c = obj.vertices[f.v3.first];
            Normal n_a = obj.normals[f.v1.second];
            Normal n_b = obj.normals[f.v2.second];
            Normal n_c = obj.normals[f.v3.second];
            if (mode == 0) {
                gouraudShading(a, b, c, n_a, n_b, n_c, obj, lights, cam, p, 
                            img);
            }
            else if (mode == 1) {
                phongShading(a, b, c, n_a, n_b, n_c, obj, lights, cam, p, img);
            }
        }
    }
    img.outputGrid(); 
    return 0;
}