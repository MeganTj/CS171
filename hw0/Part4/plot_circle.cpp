#include <fstream>
#include <iostream> 
#include <string> 
#include <stdlib.h>
#include <cmath>

using namespace std;

/* Takes in the x and y resolution of the output image and outputs a circle 
 * centered on a different colored background to standard output as a PPM image 
 */
void outputCircle(int xres, int yres) {
    cout <<  "P3" << endl; 
    cout << xres << " " << yres << endl;
    cout << 255 << endl;
    int left = -xres / 2;
    int right = xres / 2;
    int bottom = -yres / 2;
    int top = yres / 2;
    int red1 = 150; int green1 = 0; int blue1 = 75;
    int red2 = 0; int green2 = 75;int blue2 = 150; 
    int rad = min(xres, yres) / 4;
    for (int y = top; y >= bottom; y--) {
        for (int x = left; x <= right; x++) {
            if  (x != 0 && y != 0) {
                if (pow(x, 2) + pow(y, 2) <= pow(rad, 2)) {
                    cout << red1 << " " << green1 << " " << blue1 << endl;   
                }
                else {
                    cout << red2 << " " << green2 << " " << blue2 << endl;   
                }
            }
        }
    }
}

/* Assumes two positive even integers are given as input from standard input
 */
int main(int argc, char** argv) {
    // Take in the x and y resolutions
    int xres = atoi(argv[1]);
    int yres = atoi(argv[2]);
    outputCircle(xres, yres);
    return 0;
}