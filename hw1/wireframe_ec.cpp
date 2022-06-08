#include "helper.h"
#include "renderer.h"

/* Convert Cartesian NDC coordinates to a yres by xres pixel grid. Ignore the
 * z coordinates in the mapping
 */
vector<int> convertToScreen(Vector4d vec, int xres, int yres) {
    vector<int> new_vec(2);
    float nx = xres / 2;
    float ny = yres / 2;
    new_vec[0] = (vec[0] + 1.0) * nx;
    new_vec[1] = (-vec[1] + 1.0) * ny;
    return new_vec;
}

vector<vector<int>> convertToNDC(Object obj, Camera c, Perspective p, 
                        int xres, int yres, vector<vector<double>>& pixelGrid) {
    Matrix4d prod = obj.matrices[0];
    for (int i = 1; i < obj.matrices.size(); i++) {
        Matrix4d temp = obj.matrices[i] * prod;
        prod = temp;
    }
    vector<vector<int>> coords(obj.vertices.size(), vector<int>(0));
    for (int i = 1; i < obj.vertices.size(); i++) {
        Vertex v = obj.vertices[i];
        Vector4d vec;
        vec << v.x, v.y, v.z, 1;
        Vector4d nvec =  c.getWorldToCamera() * prod * vec;
        Vector4d ndc_vec = p.getProjection() * nvec;
        Vector4d scaled_vec = ndc_vec / ndc_vec[3];
        // Check that a given point is mapped within the perspective cube 
        if (abs(scaled_vec[0]) < 1 && abs(scaled_vec[1]) < 1) {
            vector<int> res = convertToScreen(scaled_vec, xres, yres);
            pixelGrid[res[1]][res[0]] = 1.0;
            coords[i] = res;
        }
    }
    return coords;
}

/* Takes in an object and outlines each of its faces using the generalized
 * bresenham formula
 */
void drawWireframe(Object obj, vector<vector<int>> coords, 
                vector<vector<double>>& pixelGrid) {
    for (Face f : obj.faces) {
        if (coords[f.v1].size() != 0 && coords[f.v2].size() != 0) {
            bresenhamGen(coords[f.v1][0], coords[f.v1][1], coords[f.v2][0], 
                        coords[f.v2][1], pixelGrid);
        }
        if (coords[f.v2].size() != 0 && coords[f.v3].size() != 0) {
            bresenhamGen(coords[f.v2][0], coords[f.v2][1], coords[f.v3][0], 
                        coords[f.v3][1], pixelGrid);
        }
        if (coords[f.v1].size() != 0 && coords[f.v3].size() != 0) {
            bresenhamGen(coords[f.v1][0], coords[f.v1][1], coords[f.v3][0], 
                        coords[f.v3][1], pixelGrid);
        }
    }
}

/* Implements the Bresenham line algorithm for the first octant (e.g. lines 
 * whose slope m is 0 <= m <= 1. Assumes that x1 > x0. Modifies the passed in 
 * pixel grid by setting "true" if a x-y position is to be filled
 * in. 
 * 
 * reflect is true if we should fill a pixel after reflecting the current
 * position across the y = x line. rotate is true if we should rotate the 
 * current position 90 degrees clockwise in the following manner: 
 * (x, y) -> (-y, x).
 */
void bresenhamFirst(int x0, int y0, int x1, int y1, 
                            vector<vector<double>>& pixelGrid, 
                            bool reflect, bool rotate) {
    int e = 0;
    int y = y0;
    float newy = y;
    int dx = x1 - x0;
    int dy = y1 - y0;
    for (int x = x0; x < x1; x++) {
        int posx = y;
        int posy = x;
        if (reflect) {
            int tempx = posx;
            posx = posy;
            posy = tempx;
        }
        if (rotate) {
            int tempx = posx;
            posx = -posy;
            posy = tempx;
        }
        float m  = dy/dx;
        int yi = floor(newy);
        float f = newy - yi;
        pixelGrid[posx][posy] = 1.0 - f;
        pixelGrid[posx][posy + 1] = f;
        if (2 * (e + dy) < dx) {
            e = e + dy;
        }
        else {
            e = e + dy - dx;
            y = y + 1;
        }
        newy += m;
    }
}

/* Implements the generalized Bresenham line algorithm by calling the line  
 * algorithm for the first octant. 
 */
void bresenhamGen(int x0, int y0, int x1, int y1, 
                            vector<vector<double>>& pixelGrid) {
    if (x0 > x1) {
        // Swap the two points so that the point with the greater x-coordinate
        // is labeled 1, and the other is labeled 0
        int tx = x0;
        int ty = y0;
        x0 = x1;
        y0 = y1;
        x1 = tx;
        y1 = ty;
    }
    // dx is guaranteed to be positive
    int dx = x1 - x0;
    int dy = y1 - y0;
    // First check if the sign of dy, as it is the sign of the slope. 
    if (dy >= 0) {
        if (dy <= dx) {
            // If dy is less than or equal to dx, the slope is between 0 and 
            // 1. The first octant bresenham algorithm works fine.
            bresenhamFirst(x0, y0, x1, y1, pixelGrid, false, false);
        }
        else {
            // Otherwise, the slope is positive, but greater than 1. 
            // In this case, we reflect the coordinates across the y = x 
            // line by swapping the x and y coordinates. Then, we let 
            // bresenhamPos know that the coordinates have been reflected, so 
            // before actually filling in the pixel grid, it'll reflect the 
            // computed coordinates back across the y = x. (x, y) -> (y, x)
            bresenhamFirst(y0, x0, y1, x1, pixelGrid, true, false);
        }
    }
    else {
        if (abs(dy) <= dx) {
            // The slope is negative, but no less than -1. To transform this 
            // slope to a positive one, we first rotate the points 90 degrees
            // counter-clockwise, which is guaranteed to give us a slope that is
            // >= 1. We then reflect these rotated coordinates across the y = x
            // line to get a slope that is <= 1. 
            // Altogether, we have: (x, y) -> (-y, x) -> (x, -y). 
            bresenhamFirst(x0, -y0, x1, -y1, pixelGrid, true, true);
        }
        else {
            // The slope is negative and less than -1. Thus, rotating 
            // coordinates 90 degrees will get us a positive slope that is <= 1.
            // (x, y) -> (-y, x)
            bresenhamFirst(-y0, x0, -y1, x1, pixelGrid, false, true);
        }
    }
}


/* Outputs the pixel grid to standard output as a .ppm image file
 */
void outputGrid(vector<vector<double>> pixelGrid, int xres, int yres) {
    cout <<  "P3" << endl; 
    cout << xres << " " << yres << endl;
    cout << 255 << endl;
    // Set the color of the vertices and wireframe lines
    int red1 = 0; int green1 = 255; int blue1 = 0; 
    for (int y = 0; y < pixelGrid.size(); y++) {
        for (int x = 0; x < pixelGrid[0].size(); x++) {
            double intensity = pixelGrid[y][x];
            cout << intensity * red1 << " " << intensity * green1 << " "
                << intensity * blue1 << endl;
        }
    }
    return;
}

int main(int argc, char** argv) {
    // Store vertices and faces so that we 
    // can copy them when needed
    unordered_map<string, vector<Vertex>> objVertices;
    unordered_map<string, vector<Face>> objFaces;
    fstream myfile("data/" + string(argv[1]));
    vector<Object> objects;
    Camera c;
    Perspective p;
    // Read in the scene description
    readScene(myfile, c, p, objects, objVertices, objFaces); 
    // Read in transformations to the objects
    readTransforms(myfile, objects, objVertices, objFaces);
    myfile.close();
    int xres = atoi(argv[2]);
    int yres = atoi(argv[3]);
    vector<vector<double>> pixelGrid(yres, vector<double>(xres)); 
    // Draw the wireframe for each object 
    for (Object obj : objects) {
        vector<vector<int>> coords = convertToNDC(obj, c, p, xres, yres, 
                                                pixelGrid);
        drawWireframe(obj, coords, pixelGrid);
    }
    outputGrid(pixelGrid, xres, yres);
    return 0;
}
