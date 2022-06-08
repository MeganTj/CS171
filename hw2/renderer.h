#include <Eigen/Dense>

void readScene(fstream& myfile, Camera& c, Perspective& p, vector<Object>& objects, 
                    unordered_map<string, vector<Vertex>>& objVertices, 
                    unordered_map<string, vector<Face>>& objFaces);
vector<int> convertToScreen(Vector4d vec, int xres, int yres);
float baryHelper(float xi, float yi, float xj, float yj, float x, float y);
float getAlpha(float xa, float ya, float xb, float yb, float xc, float yc, float x, float y);
float getBeta(float xa, float ya, float xb, float yb, float xc, float yc, float x, float y);
float getGamma(float xa, float ya, float xb, float yb, float xc, float yc, float x, float y);
Color lighting(Vertex vec, Normal n, Object obj, vector<Light> lights, Camera cam,  Perspective p);
void rasterColoredTriangle(Vector3d ndc_a, Vector3d ndc_b, Vector3d ndc_c, Color c_a, Color c_b, Color c_c, Image& img);
void gouraudShading(Vertex a, Vertex b, Vertex c, Normal n_a, Normal n_b, Normal n_c, Object obj, vector<Light> lights, Camera cam, Perspective p, Image& img);
void phongShading(Vertex a, Vertex b, Vertex c, Normal n_a, Normal n_b, Normal n_c, Object obj, vector<Light> lights, Camera cam, Perspective p, Image& img);
