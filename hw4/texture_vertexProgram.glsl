/* Sets the texture coordinate, transforms position, and transforms light 
 * directions from camera space to surface space.
 */
 
#define MAX_LIGHTS 8
attribute vec3 tangent;
varying vec3 camPos;
varying vec3 lightDir[MAX_LIGHTS];
uniform sampler2D heightmap;

void main()
{   
    gl_TexCoord[0] = gl_MultiTexCoord0;
    vec3 N = normalize(gl_NormalMatrix * gl_Normal);
    vec3 T = normalize(gl_NormalMatrix * tangent);
    vec3 B = normalize(cross(N, T));

    // Then transform camera and light vectors to surface space
    mat3 tbnMat;
    tbnMat[0] = T;
    tbnMat[1] = B;
    tbnMat[2] = N;
    vec3 viewPos = vec3(gl_ModelViewMatrix * gl_Vertex);
    for (int i = 0; i < MAX_LIGHTS; i++){
        lightDir[i] = vec3(gl_LightSource[i].position)-viewPos;
        // Because matrices are filled column order, we multiply lightDir first
        lightDir[i] = lightDir[i] * tbnMat;
    }
    gl_Position = ftransform();
}
