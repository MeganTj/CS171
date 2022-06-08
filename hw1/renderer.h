
#include <Eigen/Dense>

vector<int> convertToScreen(Vector4d vec, int xres, int yres);
vector<vector<int>> convertToNDC(Object obj, Camera c, Perspective p, 
                        int xres, int yres, vector<vector<double>>& pixelGrid);
void drawWireframe(Object obj, vector<vector<int>> coords, 
                vector<vector<double>>& pixelGrid);
void bresenhamFirst(int x0, int y0, int x1, int y1, 
                            vector<vector<double>>& pixelGrid, 
                            bool reflect);
void bresenhamGen(int x0, int y0, int x1, int y1, 
                            vector<vector<double>>& pixelGrid);
void outputGrid(vector<vector<double>> pixelGrid, int xres, int yres);
